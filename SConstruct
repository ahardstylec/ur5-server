# Construction environment
env = Environment(CPPDEFINES='DEBUG', CCFLAGS='-g -O2 -Wall')

#
# Check for python-dev
#
conf = Configure(env)
if conf.CheckCHeader('python2.4/Python.h'):
    env['PYTHON_LIB'] = 'python2.4'
elif conf.CheckCHeader('python2.5/Python.h'):
    env['PYTHON_LIB'] = 'python2.5'
elif conf.CheckCHeader('python2.6/Python.h'):
    env['PYTHON_LIB'] = 'python2.6'
elif conf.CheckCHeader('python2.7/Python.h'):
    env['PYTHON_LIB'] = 'python2.7'
else:
    print "python2.{4-7}-dev must be installed"
    Exit(1)
env = conf.Finish()

# Build '.'
env.SConscript(dirs = '.', exports = 'env')
