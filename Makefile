# Top level makefile
# Decends into SUBDIRS and runs make

SUBDIRS = kinect
all:
	@for i in $(SUBDIRS); do \
	echo "Making all in $$i..."; \
	(cd $$i; make all); done
 
clean:
	@for i in $(SUBDIRS); do \
	echo "Making clean in $$i..."; \
	(cd $$i; make clean); done
 