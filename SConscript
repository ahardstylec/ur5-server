# Import construction environment
Import('env')
# cppath and libpath has to be adjusted to match the library directory of librobotcomm
cpppath = ['../libs']
libpath = ['../libs']
libs = ['robotcomm', 'kinematics', 'configuration', 'dev', 'collision', 'm', 'math', 'pthread']
ccflags = ['-O2', '-Wall', '-g']
env = Environment(CC = 'g++',
		  CPPPATH = cpppath,
		  LIBPATH = libpath,
		  LIBS = libs,
		  CCFLAGS = ccflags,
		  CPPDEFINES = 'NDEBUG')

base_src = ['../libs/base_utils.c', '../libs/startup_utils.c', '../libs/interrupt_utils.c', '../libs/ur5lib.c', '../libs/helper.c', '../libs/tcphelper.c']
tcp_src = ['main.c'] + base_src

tcp_prog = env.Program(target='ur5-server', source=tcp_src)

env.Default(tcp_prog)

###############################################################################
