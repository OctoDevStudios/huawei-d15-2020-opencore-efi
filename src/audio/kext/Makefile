KEXT_NAME = MateBookAudio
KEXT_BUNDLE = $(KEXT_NAME).kext

SDKROOT = /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk
KERNEL_FW = $(SDKROOT)/System/Library/Frameworks/Kernel.framework

CC = clang
CXX = clang++
AS = clang
ARCH = x86_64
DEPLOYMENT_TARGET = 14.0

CXXFLAGS = \
	-arch $(ARCH) \
	-target $(ARCH)-apple-macos$(DEPLOYMENT_TARGET) \
	-isysroot $(SDKROOT) \
	-I$(KERNEL_FW)/Headers \
	-I$(KERNEL_FW)/Headers/IOKit \
	-I./$(KEXT_NAME) \
	-mkernel \
	-nostdlib \
	-std=c++17 \
	-fno-builtin \
	-fno-common \
	-fno-exceptions \
	-fno-rtti \
	-fno-stack-protector \
	-DKERNEL \
	-DKERNEL_PRIVATE \
	-DDRIVER_PRIVATE \
	-DAPPLE \
	-DNeXT \
	-D__MACOSX__ \
	-O2 \
	-Wall \
	-Wno-inconsistent-missing-override \
	-Wno-deprecated-declarations

ASFLAGS = \
	-arch $(ARCH) \
	-target $(ARCH)-apple-macos$(DEPLOYMENT_TARGET)

LDFLAGS = \
	-arch $(ARCH) \
	-target $(ARCH)-apple-macos$(DEPLOYMENT_TARGET) \
	-isysroot $(SDKROOT) \
	-Xlinker -kext \
	-undefined dynamic_lookup \
	-nostdlib \
	-lcc_kext

SRCDIR = $(KEXT_NAME)
SRCS = $(SRCDIR)/MateBookAudioDevice.cpp $(SRCDIR)/ES8336Codec.cpp
OBJS = $(SRCS:.cpp=.o) $(SRCDIR)/sof_firmware_embed.o

all: $(KEXT_BUNDLE)

$(KEXT_BUNDLE): $(OBJS)
	@echo "=== Linking $(KEXT_NAME) ==="
	mkdir -p $(KEXT_BUNDLE)/Contents/MacOS
	$(CXX) $(LDFLAGS) -o $(KEXT_BUNDLE)/Contents/MacOS/$(KEXT_NAME) $(OBJS)
	cp $(SRCDIR)/Info.plist $(KEXT_BUNDLE)/Contents/
	@echo "=== Build complete: $(KEXT_BUNDLE) ==="

%.o: %.cpp
	@echo "=== Compiling $< ==="
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(SRCDIR)/sof_firmware_embed.o: $(SRCDIR)/sof_firmware_embed.s $(SRCDIR)/Firmware/sof-cml.ri
	@echo "=== Embedding firmware ==="
	$(AS) $(ASFLAGS) -c $< -o $@

clean:
	rm -rf $(KEXT_BUNDLE) $(OBJS)

.PHONY: all clean
