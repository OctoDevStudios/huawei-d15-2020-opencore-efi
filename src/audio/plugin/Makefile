PLUGIN_NAME = MateBookAudioPlugin
BUNDLE = MateBookAudio.driver

SDKROOT = /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk

CC = clang
ARCH = x86_64
CFLAGS = \
	-arch $(ARCH) \
	-target $(ARCH)-apple-macos14.0 \
	-isysroot $(SDKROOT) \
	-O2 -Wall \
	-framework CoreFoundation \
	-framework CoreAudio \
	-framework IOKit

all: $(BUNDLE)

$(BUNDLE): $(PLUGIN_NAME).c Info.plist
	@echo "=== Building $(BUNDLE) ==="
	mkdir -p $(BUNDLE)/Contents/MacOS
	$(CC) $(CFLAGS) -bundle -o $(BUNDLE)/Contents/MacOS/$(PLUGIN_NAME) $(PLUGIN_NAME).c
	cp Info.plist $(BUNDLE)/Contents/
	@echo "=== Done: $(BUNDLE) ==="

clean:
	rm -rf $(BUNDLE)

install: $(BUNDLE)
	sudo mkdir -p /Library/Audio/Plug-Ins/HAL
	sudo cp -r $(BUNDLE) /Library/Audio/Plug-Ins/HAL/
	sudo killall coreaudiod

uninstall:
	sudo rm -rf /Library/Audio/Plug-Ins/HAL/$(BUNDLE)
	sudo killall coreaudiod

.PHONY: all clean install uninstall
