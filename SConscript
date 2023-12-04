from building import *

cwd = GetCurrentDir()
path = [cwd+'/inc']
src  = Glob('src/*.c')
 
group = DefineGroup('bt_mx01', src, depend = ['PKG_USING_BT_MX01'], CPPPATH = path)

Return('group')