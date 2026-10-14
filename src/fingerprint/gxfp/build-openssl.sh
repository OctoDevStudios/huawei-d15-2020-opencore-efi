#!/bin/zsh
# Builds OpenSSL 3.3.2 statically into ../deps/ossl (required by the gxfp Makefile).
set -e
D=${0:A:h}/../deps; mkdir -p $D && cd $D
[[ -f ossl/lib/libssl.a ]] && { echo "already built"; exit 0; }
curl -sLO https://github.com/openssl/openssl/releases/download/openssl-3.3.2/openssl-3.3.2.tar.gz
tar xzf openssl-3.3.2.tar.gz && cd openssl-3.3.2
./Configure darwin64-x86_64-cc no-shared no-tests no-docs --prefix=$D/ossl
make -j8 build_libs && make install_dev
