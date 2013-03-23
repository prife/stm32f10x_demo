import os
import sys
import rtconfig

if os.getenv('RTT_ROOT'):
    RTT_ROOT = os.getenv('RTT_ROOT')
else:
    RTT_ROOT = os.path.normpath(os.getcwd() + '/../..')

sys.path = sys.path + [os.path.join(RTT_ROOT, 'tools')]
from building import *

TARGET = 'rtthread-stm32.' + rtconfig.TARGET_EXT

env = Environment(tools = ['mingw'],
	AS = rtconfig.AS, ASFLAGS = rtconfig.AFLAGS,
	CC = rtconfig.CC, CCFLAGS = rtconfig.CFLAGS,
	AR = rtconfig.AR, ARFLAGS = '-rc',
	LINK = rtconfig.LINK, LINKFLAGS = rtconfig.LFLAGS)
env.PrependENVPath('PATH', rtconfig.EXEC_PATH)

if rtconfig.PLATFORM == 'iar':
	env.Replace(CCCOM = ['$CC $CCFLAGS $CPPFLAGS $_CPPDEFFLAGS $_CPPINCFLAGS -o $TARGET $SOURCES'])
	env.Replace(ARFLAGS = [''])
	env.Replace(LINKCOM = ['$LINK $SOURCES $LINKFLAGS -o $TARGET --map project.map'])

def mdk_create_tmpfile(tmpfile, objs):
    cmdline =''
    tmpfile = file(tmpfile, 'w')
    for item in objs:
        # print type(item), os.path.basename(str(item))
        cmdline += os.path.normpath(str(item))
        cmdline += ' '

    tmpfile.write(cmdline)
    tmpfile.close();
    return

if rtconfig.PLATFORM == 'armcc':
    env["LINKCOM"] = "$LINK -o $TARGET $LINKFLAGS --via tmpcmd.txt"

Export('RTT_ROOT')
Export('rtconfig')

# prepare building environment
objs = PrepareBuilding(env, RTT_ROOT)

# STM32 firemare library building script
# objs = objs + SConscript( GetCurrentDir() + '/Libraries/SConscript', variant_dir='build/bsp/Libraries', duplicate=0)

if GetDepend('RT_USING_RTGUI'):
    objs = objs + SConscript(RTT_ROOT + '/examples/gui/SConscript', variant_dir='build/examples/gui', duplicate=0)

# build program 
if rtconfig.PLATFORM == 'armcc':
	mdk_create_tmpfile('tmpcmd.txt', objs)

prog = env.Program(TARGET, objs)

# end building 
EndBuilding(TARGET, prog)
