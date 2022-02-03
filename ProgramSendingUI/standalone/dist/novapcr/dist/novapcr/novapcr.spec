# -*- mode: python ; coding: utf-8 -*-
from kivy_deps import sdl2, glew

block_cipher = None


a = Analysis(['connection.py', 'gui.py', 'main.py', 'readfile.py'],
             pathex=['C:\\Users\\fbcat\\Documents\\CEDOC\\UI\\'],
             binaries=[],
             datas=[('C:\\Users\\fbcat\\Documents\\CEDOC\\UI\\', 'UI'), ('C:\\Users\\fbcat\\Documents\\CEDOC\\UI\\venv\\Lib\\site-packages\\win32\\lib\\win32timezone.py', '.')],
             hiddenimports=[],
             hookspath=[],
             hooksconfig={},
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher,
             noarchive=False)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)

exe = EXE(pyz,
          a.scripts, 
          [],
          exclude_binaries=True,
          name='novapcr',
          debug=False,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          console=True,
          disable_windowed_traceback=False,
          target_arch=None,
          codesign_identity=None,
          entitlements_file=None )
coll = COLLECT(exe, Tree('C:\\Users\\fbcat\\Documents\\CEDOC\\UI\\'),
               a.binaries,
               a.zipfiles,
               a.datas, 
               *[Tree(p) for p in (sdl2.dep_bins + glew.dep_bins)],
               strip=False,
               upx=True,
               upx_exclude=[],
               name='novapcr')
