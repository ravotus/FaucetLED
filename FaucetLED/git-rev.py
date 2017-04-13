import os
from subprocess import PIPE, run
import time

thisdir = os.path.dirname(__file__)
version_c = os.path.join(thisdir, 'Src', 'versions.c')

git = run(['git', 'describe', '--dirty', '--always', '--tags'], check=True, stdout=PIPE)
revision = git.stdout.decode('ascii').strip()

with open(version_c, 'w') as f:
    f.write('/* Note: Don\'t build this file with -flto, otherwise the names of\n')
    f.write(' * these variables will not be present in the map file and will be\n')
    f.write(' * optimized out. */\n\n')
    f.write('const char GIT_REVISION[] __attribute__((section(".revision"))) = "{}";\n'.format(revision))
    f.write('const char BUILD_DATE[] __attribute__((section(".revision"))) = "{}";\n'.format(time.strftime("%c")))
