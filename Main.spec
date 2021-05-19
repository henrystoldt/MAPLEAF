# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(['MAPLEAF\\Main.py'],
             pathex=['C:\\Users\\rando\\Documents\\MAPLEAF'],
             binaries=[],
             datas=[ 
                ('MAPLEAF/Examples/Wind/*', 'MAPLEAF/Examples/Wind/'),
                ('MAPLEAF/Examples/Simulations/*', 'MAPLEAF/Examples/Simulations/'),
                ('MAPLEAF/Examples/Motors/*', 'MAPLEAF/Examples/Motors/'),
                ('MAPLEAF/Examples/TabulatedData/*', 'MAPLEAF/Examples/TabulatedData/'),
                ('MAPLEAF/ENV/*.txt', 'MAPLEAF/ENV'),
                ('MAPLEAF/IO/*.jpg', 'MAPLEAF/IO'),
              ],
             hiddenimports=[],
             hookspath=[],
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
          a.binaries,
          a.zipfiles,
          a.datas,
          [],
          name='Main',
          debug=False,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          upx_exclude=[],
          runtime_tmpdir=None,
          console=True )
