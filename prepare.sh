#!/bin/sh
scripts/make_headers.py definitions/api.xml include/APHY
scripts/make_headers_cpp.py definitions/api.xml include/APHY
# scripts/make_icdloader.py definitions/api.xml implementations/Loader
