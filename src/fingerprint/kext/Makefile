SDKROOT = /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk
KFW = $(SDKROOT)/System/Library/Frameworks/Kernel.framework
FLAGS = -arch x86_64 -target x86_64-apple-macos14.0 -isysroot $(SDKROOT)
CXXFLAGS = $(FLAGS) -I$(KFW)/Headers -mkernel -nostdlib -std=c++17 -fno-builtin -fno-common \
	-fno-exceptions -fno-rtti -fno-stack-protector -DKERNEL -DKERNEL_PRIVATE -DDRIVER_PRIVATE \
	-DAPPLE -DNeXT -O2 -Wall -Wno-inconsistent-missing-override -Wno-deprecated-declarations
LDFLAGS = $(FLAGS) -Xlinker -kext -undefined dynamic_lookup -nostdlib -lcc_kext

MateBookFP.kext: MateBookFP/MateBookFP.cpp MateBookFP/Info.plist
	clang++ $(CXXFLAGS) -c MateBookFP/MateBookFP.cpp -o MateBookFP/MateBookFP.o
	mkdir -p MateBookFP.kext/Contents/MacOS
	clang++ $(LDFLAGS) MateBookFP/MateBookFP.o -o MateBookFP.kext/Contents/MacOS/MateBookFP
	cp MateBookFP/Info.plist MateBookFP.kext/Contents/Info.plist

clean:
	rm -rf MateBookFP.kext MateBookFP/*.o
