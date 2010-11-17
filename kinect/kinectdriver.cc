/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2003
 *     Brian Gerkey
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_kinect kinect
 * @brief USB camera driver for Microsoft Kinect camera

%Device driver for Microsoft Kinect webcams.  This driver handles processing
the RGB Color images and the greyscale depth images provided by the Kinect.
The Kinect also supports motor control for PTZ, accelerometers, audio and
LED control, these capabilities are under development.

@par Compile-time dependencies

- none

@par Requires

- none

@par Provides

- @ref interface_camera - Color image (mandatory)
- @ref interface_camera - Depth image (optional)

@par Configuration requests

- none

@par Configuration file options

- heatmap (bool)
  - Default: false
  - When set to false, the Depth image is published as a greyscale MONO16 image
  - When set to true, the Depth image is colorized and published as an RGB heatmap

@par Example

@verbatim
driver
(
  name "kinect"
  provides ["color:::camera:0" "depth:::camera:1"]
  heatmap 1
)
@endverbatim

@authors Rich Mattes
 */
/** @} */

#if !defined (WIN32)
#include <unistd.h>
#endif
#include <string.h>
#include <cmath>

#include <libplayercore/playercore.h>
#include <libusb-1.0/libusb.h>
#include <libfreenect/libfreenect.h>


// Callback functions for processing depth and color image buffers
void DepthImageCallback(uint16_t *imagedata, int width, int height);
void ColorImageCallback(uint8_t *imagedata, int width, int height);

// Storage for image data and metadata
static uint16_t* DepthImage;
static uint8_t* ColorImage;
static pthread_mutex_t kinect_mutex;
static int cwidth, cheight, dwidth, dheight, newcdata, newddata;

////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class KinectDriver : public ThreadedDriver
{
public:

	// Constructor; need that
	KinectDriver(ConfigFile* cf, int section);

	// This method will be invoked on each incoming message
	virtual int ProcessMessage(QueuePointer &resp_queue,
			player_msghdr * hdr,
			void * data);

private:

	// Main function for device thread.
	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();
	int PublishColorImage();
	int PublishDepthImage();
	int SetupPTZ();

	// Handle to the kinect usb object
	libusb_device_handle *usbdev;
	libusb_device_handle *ptzdev;

	// Device addresses for all possible provided interfaces
	player_devaddr_t color_camera_id;
	player_devaddr_t depth_camera_id;
	player_devaddr_t ptz_id;
	player_devaddr_t imu_id;

	// Storage for outgoing camera data
	player_camera_data_t colordata;
	player_camera_data_t depthdata;

	// Flags for interfaces we privide
	int providedepthimage;
	int provideptz;
	int provideimu;

	// Config file options
	int heatmap;
	int downsample;

	uint16_t t_gamma[2048];
};


// Startup instance of the driver
Driver* KinectDriver_Init(ConfigFile* cf, int section)
{
	// Create and return a new instance of this driver
	return((Driver*)(new KinectDriver(cf, section)));
}

// Register driver with server
void KinectDriver_Register(DriverTable* table)
{
	table->AddDriver("kinect", KinectDriver_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
KinectDriver::KinectDriver(ConfigFile* cf, int section)
: ThreadedDriver(cf, section)
{
	providedepthimage = 0;
	//Add interface from Configuration File
	if (cf->ReadDeviceAddr(&(this->color_camera_id), section, "provides", PLAYER_CAMERA_CODE, -1, "image"))
	{
		PLAYER_ERROR("Kinect's Camera interface not started: config file doesn't provide image:::camera.");
		this->SetError(-1);
		return;
	}
	if (this->AddInterface(this->color_camera_id))
	{
		PLAYER_ERROR("Kinect's Camera interface failed to be added.");
		this->SetError(-1);
		return;
	}
	if (cf->ReadDeviceAddr(&(this->depth_camera_id), section, "provides", PLAYER_CAMERA_CODE, -1, "depth"))
	{
		PLAYER_WARN("Kinect's Depth interface not started: config file doesn't provide depth:::camera.");
	}
	else{
		if (this->AddInterface(this->depth_camera_id))
		{
			PLAYER_ERROR("Kinect's Depth Camera interface failed to be added.");
			this->SetError(-1);
			return;
		}
		providedepthimage = 1;
	}

	// Check to see if we provide the PTZ interface
	if (cf->ReadDeviceAddr(&(this->ptz_id), section, "provides", PLAYER_PTZ_CODE, -1, NULL))
	{
		PLAYER_WARN("Kinect driver not providing PTZ.");
	}
	else{
		if (this->AddInterface(this->ptz_id))
		{
			PLAYER_ERROR("Kinect's PTZ interface failed to be added.");
			this->SetError(-1);
			return;
		}
		provideptz = 1;
	}

	// Check to see if we provide the IMU interface
	if (cf->ReadDeviceAddr(&(this->imu_id), section, "provides", PLAYER_IMU_CODE, -1, NULL))
	{
		PLAYER_WARN("Kinect driver not providing IMU.");
	}
	else{
		if (this->AddInterface(this->imu_id))
		{
			PLAYER_ERROR("Kinect's IMU interface failed to be added.");
			this->SetError(-1);
			return;
		}
		provideimu = 1;
	}


	// Read config file options
	heatmap = cf->ReadBool(section, "heatmap", false);
	downsample = cf->ReadBool(section, "downsample", false);

	// Assign pointers
	usbdev = NULL;
	ptzdev = NULL;

	// Initialize the colormap lookup table
	for (int i=0; i<2048; i++) {
			float v = i/2048.0;
			v = powf(v, 3)* 6;
			t_gamma[i] = v*6*256;
		}

	return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
int KinectDriver::MainSetup()
{
	PLAYER_MSG0(1,"Kinect driver initializing...");
	kinect_mutex = PTHREAD_MUTEX_INITIALIZER;

	libusb_init(NULL);
	usbdev = libusb_open_device_with_vid_pid(NULL, 0x45e, 0x2ae);
	if (!usbdev) {
		PLAYER_ERROR("Error opening connection to Kinect");
		return -1;
	}
	libusb_claim_interface(usbdev, 0);

	PLAYER_MSG0(1,"Kinect driver ready.");
	printf("Kinect device is %i\n", libusb_get_device_address(libusb_get_device(usbdev)));

	cams_init(usbdev, DepthImageCallback, ColorImageCallback);

	colordata.image = NULL;
	depthdata.image = NULL;

	cwidth = 0;
	cheight = 0;
	dwidth = 0;
	dheight = 0;

	if (provideptz || provideimu)
	{
		SetupPTZ();
	}

	return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
void KinectDriver::MainQuit()
{
	PLAYER_MSG0(2,"Kinect driver shutting down...");

	// Close and release the USB device
	libusb_close(usbdev);
	usbdev = NULL;

	if (provideptz || provideimu)
	{
		libusb_close(ptzdev);
		ptzdev = NULL;
	}

	PLAYER_MSG0(2,"Kinect driver has been shut down.");
}
////////////////////////////////////////////////////////////////////////////////
// Setup PTZ USB port
int KinectDriver::SetupPTZ()
{
	ptzdev = libusb_open_device_with_vid_pid(NULL, 0x45e, 0x2b0);
	if (!ptzdev) {
		PLAYER_ERROR("Error opening connection to Kinect");
		return -1;
	}
	libusb_claim_interface(ptzdev, 0);

	libusb_set_configuration(ptzdev, 0);

	libusb_control_transfer(ptzdev, 0xC0, 0x10, 0x0, 0x0, 0, 1, 1);
}

////////////////////////////////////////////////////////////////////////////////
// Message handling function
int KinectDriver::ProcessMessage(QueuePointer & resp_queue,
		player_msghdr * hdr,
		void * data)
{
	// This driver does not handle incoming messages, return -1
	return(-1);
}
////////////////////////////////////////////////////////////////////////////////
// Publish color image data to camera interface
int KinectDriver::PublishColorImage()
{
	if (cwidth!=640 || cheight!=480)
		return -1;

	if (colordata.image){
		delete[] colordata.image;
	}
	colordata.image = new uint8_t[cheight*cwidth*3];

	pthread_mutex_lock(&kinect_mutex);
	colordata.width = cwidth;
	colordata.height = cheight;
	memcpy(colordata.image, ColorImage, 3*cwidth*cheight);
	pthread_mutex_unlock(&kinect_mutex);

	colordata.bpp = 24;
	colordata.compression = PLAYER_CAMERA_COMPRESS_RAW;
	colordata.fdiv = 1;
	colordata.image_count = cwidth*cheight*3;
	colordata.format = PLAYER_CAMERA_FORMAT_RGB888;

	PLAYER_MSG2(4,"Writing Color Image size %d, %d", colordata.width, colordata.height);
	Publish(color_camera_id, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, (void*)&colordata);
	newcdata = 0;

	return 0;

}
////////////////////////////////////////////////////////////////////////////////
// Publish depth image data to camera interface
int KinectDriver::PublishDepthImage()
{

	if (dwidth!=640 || dheight!=480)
		return -1;

	if (depthdata.image){
		delete[] depthdata.image;
	}
	depthdata.image = new uint8_t[dheight*dwidth*3];

	// If no heatmap, just publish image as MONO16
	if (!heatmap)
	{
		if (!downsample)
		{
			pthread_mutex_lock(&kinect_mutex);
			depthdata.width = dwidth;
			depthdata.height = dheight;
			memcpy((void*)depthdata.image, (void*)DepthImage, dwidth*dheight*sizeof(uint16_t));
			pthread_mutex_unlock(&kinect_mutex);


			depthdata.bpp = 16;
			depthdata.compression = PLAYER_CAMERA_COMPRESS_RAW;
			depthdata.fdiv = 1;
			depthdata.image_count = dwidth * dheight * 2;
			depthdata.format = PLAYER_CAMERA_FORMAT_MONO16;
		}
		else
		{
			pthread_mutex_lock(&kinect_mutex);
			depthdata.width = dwidth;
			depthdata.height = dheight;
			for (int i=0; i < dwidth*dheight; i++)
			{
				uint16_t pixel = DepthImage[i];
				uint8_t dpixel = (uint8_t)((double)pixel / 2048.0 * 255.0); 
				depthdata.image[i] = dpixel;
			}
			pthread_mutex_unlock(&kinect_mutex);

			depthdata.bpp = 8;
			depthdata.compression = PLAYER_CAMERA_COMPRESS_RAW;
			depthdata.fdiv = 1;
			depthdata.image_count = dwidth * dheight;
			depthdata.format = PLAYER_CAMERA_FORMAT_MONO8;
		}
	}
	else	// We are using a heatmap, compute and publish as RGB888
	{
		pthread_mutex_lock(&kinect_mutex);
		depthdata.width = dwidth;
		depthdata.height = dheight;
		for (int i=0; i<640*480; i++) {
			int pval = t_gamma[DepthImage[i]];
			int lb = pval & 0xff;
			switch (pval>>8) {
			case 0:
				depthdata.image[3*i+0] = 255;
				depthdata.image[3*i+1] = 255-lb;
				depthdata.image[3*i+2] = 255-lb;
				break;
			case 1:
				depthdata.image[3*i+0] = 255;
				depthdata.image[3*i+1] = lb;
				depthdata.image[3*i+2] = 0;
				break;
			case 2:
				depthdata.image[3*i+0] = 255-lb;
				depthdata.image[3*i+1] = 255;
				depthdata.image[3*i+2] = 0;
				break;
			case 3:
				depthdata.image[3*i+0] = 0;
				depthdata.image[3*i+1] = 255;
				depthdata.image[3*i+2] = lb;
				break;
			case 4:
				depthdata.image[3*i+0] = 0;
				depthdata.image[3*i+1] = 255-lb;
				depthdata.image[3*i+2] = 255;
				break;
			case 5:
				depthdata.image[3*i+0] = 0;
				depthdata.image[3*i+1] = 0;
				depthdata.image[3*i+2] = 255-lb;
				break;
			default:
				depthdata.image[3*i+0] = 0;
				depthdata.image[3*i+1] = 0;
				depthdata.image[3*i+2] = 0;
				break;
			}
		}
		pthread_mutex_unlock(&kinect_mutex);

		depthdata.bpp = 24;
		depthdata.compression = PLAYER_CAMERA_COMPRESS_RAW;
		depthdata.fdiv = 1;
		depthdata.image_count = dwidth*dheight*3;
		depthdata.format = PLAYER_CAMERA_FORMAT_RGB888;
	}
	PLAYER_MSG2(4,"Writing Depth Image size %d, %d", depthdata.width, depthdata.height);
	newddata = 0;
	Publish(depth_camera_id, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, (void*)&depthdata);

	return 0;
}
////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void KinectDriver::Main()
{
	// The main loop; interact with the device here
	for(;;)
	{
		// test if we are supposed to cancel
		pthread_testcancel();

		// Cycle libusb
		libusb_handle_events(NULL);

		// Process incoming messages.  KinectDriver::ProcessMessage() is
		// called on each message.
		ProcessMessages();

		// Interact with the device, and push out the resulting data, using
		if (newcdata)
		{
			PublishColorImage();
		}
		if (providedepthimage && newddata)
		{
			PublishDepthImage();
		}

		// Sleep for a while so as not to hog the system
		usleep(10);
	}
}
////////////////////////////////////////////////////////////////////////////////
// Grab depth image buffer returned by libfreenect and store it in a static buffer
void DepthImageCallback(uint16_t *imagedata, int width, int height)
{
	pthread_mutex_lock(&kinect_mutex);
	if(DepthImage)
	{
		delete[] DepthImage;
	}

	DepthImage = new uint16_t[width*height];
	memcpy(DepthImage, imagedata, width*height*sizeof(uint16_t));
	dwidth = width;
	dheight = height;
	newddata = 1;
	pthread_mutex_unlock(&kinect_mutex);
	return;
}
////////////////////////////////////////////////////////////////////////////////
// Grab color image buffer returned by libfreenect and store it in a static buffer
void ColorImageCallback(uint8_t *imagedata, int width, int height)
{
	pthread_mutex_lock(&kinect_mutex);
	if(ColorImage)
	{
		delete[] ColorImage;
	}
	ColorImage = new uint8_t[width*height*3];
	memcpy(ColorImage, imagedata, 3*width*height);
	printf("Setting size (%d, %d)\n",width, height);
	cwidth = width;
	cheight = height;
	newcdata = 1;
	pthread_mutex_unlock(&kinect_mutex);
	return;
}

////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.

/* need the extern to avoid C++ name-mangling  */
extern "C" {
int player_driver_init(DriverTable* table)
{
	puts("Kinect driver initializing");
	KinectDriver_Register(table);
	puts("Kinect driver done");
	return(0);
}
}
