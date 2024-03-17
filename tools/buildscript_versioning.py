FILENAME_BUILDNO = '.pio/versioning'
FILENAME_VERSION_H = 'include/version.h'
version = 'v0.3.'

import datetime
from subprocess import *

build_no = 0
try:
    with open(FILENAME_BUILDNO) as f:
        build_no = int(f.readline()) + 1
except:
    print('Starting build number from 1..')
    build_no = 1
with open(FILENAME_BUILDNO, 'w+') as f:
    f.write(str(build_no))
    print('Build number: {}'.format(build_no))

version_full = version + str(build_no)

try:
    git_id = Popen('git rev-parse --short HEAD', stdout=PIPE, shell=True).stdout.read().strip().decode('ascii')
    version_full = "%s-%s" % (version_full, git_id)
except:
    #git_id = "0000000"
    git_id = "No_GIT.."
    pass

date_now = "%.16s" % datetime.datetime.now()
version_string = "{} - {}".format(version_full, date_now)

# dl9sau: build_no in base62 -> base62 (0-9, a-z, A-Z)
# This gives us room for (62**2)-1 = 3843 builds between git commits. Should be enough
# git_id: length of 5 has hopefully enough entropy.
# VERS_XXSHORT_BN may also be sent on RF -> We keep it short. 8 bytes now, instead of typically 3 bytes before.
s="0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
bnA=s[(int(build_no / len(s))) % len(s)]
bnB=s[build_no % len(s)]
vers_xxshort_bn="%.5s.%c%c" % (git_id, bnA, bnB)

hf = """
#ifndef BUILD_NUMBER
  #define BUILD_NUMBER "{}"
#endif
#ifndef VERSION
  #define VERSION "{}"
#endif
#ifndef VERSION_SHORT
  #define VERSION_SHORT "{}"
#endif
#ifndef VERS_XXSHORT_BN
  #define VERS_XXSHORT_BN "{}"
#endif
""".format(build_no, version_string, version_full, vers_xxshort_bn)
with open(FILENAME_VERSION_H, 'w+') as f:
    f.write(hf)

with open("data_embed/index.html", "r") as f:
    index_html_content = f.read()

index_html_content = index_html_content.replace('<!--VERSION-->', version_string)

with open("data_embed/index.html.out", "w") as f:
    f.write(index_html_content)
