#
# UCD_config.py - configuration for UCD generator
#
#
# Licensed under Open Software License 3.0.
#
#
UCD_version_major = 13
UCD_version_minor = 0
UCD_version_point = 0
UCD_version_string = "%i.%i.%i" % (UCD_version_major, UCD_version_minor, UCD_version_point)

UCD_src_dir = "UCD-" + UCD_version_string

UCD_output_dir = "UCD-generated-" + UCD_version_string

UCD_max_code_point = "0x10FFFF"
