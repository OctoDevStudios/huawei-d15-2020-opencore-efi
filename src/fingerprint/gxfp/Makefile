OSSL = ../deps/ossl
CFLAGS = -O2 -g -Wall -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Imac -Isrc -I$(OSSL)/include -DGLIB_SHIM
SRCS = gxfp.c src/goodix_tls.c src/gx51_target.c src/gx51_factory_pmk.c src/gx51_image.c \
       src/gx51_capture_recipe.c src/goodix_sift.c src/fastbrief/sigfm.c mac/mac_hw.c
all: gxfp gxfp-agent

gxfp-agent: agent.c
	clang -O2 -Wall -Wno-deprecated-declarations agent.c -framework ApplicationServices -framework CoreFoundation -framework IOKit -o $@
	codesign -s - -f -i com.hackintosh.gxfp-agent $@

gxfp: $(SRCS) mac/glib.h unlock.c
	clang $(CFLAGS) $(SRCS) $(OSSL)/lib/libssl.a $(OSSL)/lib/libcrypto.a -framework IOKit -framework CoreFoundation -framework Security -framework OpenDirectory -framework ApplicationServices -lm -o $@
clean:
	rm -f gxfp
