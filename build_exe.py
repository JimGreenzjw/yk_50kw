#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
构建exe文件的脚本
将can_monitor_enhanced_yk.py转换为可执行文件
"""

import os
import sys
import subprocess
import shutil
from pathlib import Path

def check_dependencies():
    """检查必要的依赖是否已安装"""
    required_packages = [
        'PyQt5',
        'pyqtgraph', 
        'numpy',
        'PyInstaller'
    ]
    
    missing_packages = []
    for package in required_packages:
        try:
            if package == 'PyInstaller':
                import PyInstaller
            else:
                __import__(package)
            print(f"✓ {package} 已安装")
        except ImportError:
            missing_packages.append(package)
            print(f"✗ {package} 未安装")
    
    if missing_packages:
        print(f"\n需要安装以下包: {', '.join(missing_packages)}")
        print("请运行以下命令安装:")
        for package in missing_packages:
            if package == 'pyinstaller':
                print(f"pip install {package}")
            else:
                print(f"pip install {package}")
        return False
    
    return True

def install_pyinstaller():
    """安装PyInstaller"""
    try:
        subprocess.run([sys.executable, '-m', 'pip', 'install', 'pyinstaller'], 
                      check=True, capture_output=True, text=True)
        print("✓ PyInstaller 安装成功")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ PyInstaller 安装失败: {e}")
        return False

def build_exe():
    """构建exe文件"""
    # 获取当前目录
    current_dir = Path(__file__).parent
    script_path = current_dir / "can_monitor_enhanced_yk.py"
    
    if not script_path.exists():
        print(f"✗ 找不到脚本文件: {script_path}")
        return False
    
    # 构建输出目录
    dist_dir = current_dir / "dist"
    build_dir = current_dir / "build"
    
    # 清理旧的构建文件
    if dist_dir.exists():
        shutil.rmtree(dist_dir)
        print("✓ 清理旧的dist目录")
    
    if build_dir.exists():
        shutil.rmtree(build_dir)
        print("✓ 清理旧的build目录")
    
    # 构建PyInstaller命令
    cmd = [
        sys.executable, '-m', 'PyInstaller',
        '--onefile',                    # 打包成单个exe文件
        '--windowed',                   # 不显示控制台窗口
        '--name=can_monitor_yk',        # 指定exe文件名
        '--icon=icon.ico',              # 图标文件（如果存在）
        '--add-data=kerneldlls;kerneldlls',  # 包含kerneldlls目录
        '--add-data=zlgcan.dll;.',     # 包含zlgcan.dll
        '--add-data=zlgcan.py;.',      # 包含zlgcan.py
        '--hidden-import=PyQt5.QtCore',
        '--hidden-import=PyQt5.QtGui', 
        '--hidden-import=PyQt5.QtWidgets',
        '--hidden-import=pyqtgraph',
        '--hidden-import=numpy',
        '--hidden-import=zlgcan',
        '--collect-all=pyqtgraph',
        '--collect-all=PyQt5',
        str(script_path)
    ]
    
    # 如果图标文件不存在，移除图标参数
    icon_path = current_dir / "icon.ico"
    if not icon_path.exists():
        cmd = [arg for arg in cmd if not arg.startswith('--icon')]
        print("⚠ 图标文件不存在，将使用默认图标")
    
    print("开始构建exe文件...")
    print("注意: 构建过程可能需要几分钟时间，请耐心等待")
    print(f"命令: {' '.join(cmd)}")
    
    try:
        # 运行PyInstaller
        print("正在构建，请耐心等待...")
        result = subprocess.run(cmd, check=True, text=True)
        print("✓ exe文件构建成功!")
        
        # 检查输出文件
        exe_path = dist_dir / "can_monitor_yk.exe"
        if exe_path.exists():
            file_size = exe_path.stat().st_size / (1024 * 1024)  # MB
            print(f"✓ 生成的文件: {exe_path}")
            print(f"✓ 文件大小: {file_size:.1f} MB")
            
            # 复制必要的文件到dist目录
            copy_additional_files(current_dir, dist_dir)
            
            return True
        else:
            print("✗ 未找到生成的exe文件")
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"✗ 构建失败: {e}")
        print(f"错误输出: {e.stderr}")
        return False

def copy_additional_files(source_dir, dist_dir):
    """复制额外的必要文件到dist目录"""
    files_to_copy = [
        "kerneldlls",
        "zlgcan.dll", 
        "zlgcan.py"
    ]
    
    print("\n复制额外文件...")
    for file_name in files_to_copy:
        source_path = source_dir / file_name
        dest_path = dist_dir / file_name
        
        if source_path.exists():
            if source_path.is_dir():
                if dest_path.exists():
                    shutil.rmtree(dest_path)
                shutil.copytree(source_path, dest_path)
            else:
                shutil.copy2(source_path, dest_path)
            print(f"✓ 复制: {file_name}")
        else:
            print(f"⚠ 文件不存在: {file_name}")

def create_batch_file(dist_dir):
    """创建启动批处理文件"""
    batch_content = """@echo off
echo 启动CAN监控程序...
echo.
echo 请确保以下文件存在:
echo - can_monitor_yk.exe
echo - kerneldlls/ 目录
echo - zlgcan.dll
echo - zlgcan.py
echo.
pause
start can_monitor_yk.exe
"""
    
    batch_path = dist_dir / "启动CAN监控.bat"
    with open(batch_path, 'w', encoding='gbk') as f:
        f.write(batch_content)
    
    print(f"✓ 创建启动脚本: {batch_path}")

def create_readme(dist_dir):
    """创建说明文件"""
    readme_content = """# CAN监控程序使用说明

## 文件说明
- can_monitor_yk.exe: 主程序文件
- kerneldlls/: CAN设备驱动文件目录
- zlgcan.dll: ZLG CAN库文件
- zlgcan.py: ZLG CAN Python接口
- 启动CAN监控.bat: 启动脚本

## 使用步骤
1. 确保所有文件在同一目录下
2. 双击"启动CAN监控.bat"或直接运行"can_monitor_yk.exe"
3. 在程序界面中选择设备类型和参数
4. 点击"打开设备"连接CAN设备
5. 点击"开始接收"开始监控数据

## 注意事项
- 首次运行可能需要安装Visual C++ Redistributable
- 确保CAN设备已正确连接
- 程序需要管理员权限才能访问硬件设备

## 故障排除
如果程序无法启动，请检查:
1. 是否安装了Visual C++ Redistributable
2. 是否以管理员身份运行
3. 所有必要文件是否完整
4. 防病毒软件是否阻止了程序运行

## 技术支持
如有问题，请联系技术支持。
"""
    
    readme_path = dist_dir / "使用说明.txt"
    with open(readme_path, 'w', encoding='utf-8') as f:
        f.write(readme_content)
    
    print(f"✓ 创建说明文件: {readme_path}")

def main():
    """主函数"""
    print("=" * 50)
    print("CAN监控程序 - exe构建工具")
    print("=" * 50)
    
    # 检查依赖
    print("\n1. 检查依赖...")
    if not check_dependencies():
        print("\n是否自动安装PyInstaller? (y/n): ", end="")
        if input().lower() == 'y':
            if install_pyinstaller():
                if not check_dependencies():
                    print("✗ 依赖检查失败，请手动安装缺少的包")
                    return
            else:
                print("✗ PyInstaller安装失败")
                return
        else:
            print("请手动安装缺少的依赖包后重新运行")
            return
    
    # 构建exe
    print("\n2. 构建exe文件...")
    if build_exe():
        print("\n3. 创建额外文件...")
        
        # 创建启动脚本和说明文件
        dist_dir = Path(__file__).parent / "dist"
        create_batch_file(dist_dir)
        create_readme(dist_dir)
        
        print("\n" + "=" * 50)
        print("✓ 构建完成!")
        print(f"✓ 输出目录: {dist_dir}")
        print("✓ 可以运行以下文件:")
        print(f"  - {dist_dir / 'can_monitor_yk.exe'}")
        print(f"  - {dist_dir / '启动CAN监控.bat'}")
        print("=" * 50)
        
        # 询问是否打开输出目录
        print("\n是否打开输出目录? (y/n): ", end="")
        if input().lower() == 'y':
            os.startfile(dist_dir)
    else:
        print("\n✗ 构建失败，请检查错误信息")

if __name__ == "__main__":
    main() 