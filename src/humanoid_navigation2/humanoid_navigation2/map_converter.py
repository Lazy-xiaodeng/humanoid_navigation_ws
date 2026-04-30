#!/usr/bin/env python3
import json
import yaml
import os
import sys
import shutil

def convert_and_move_map(json_path):
    # 1. 定义目标路径（humanoid_navigation2 的 maps 文件夹）
    # 获取脚本所在目录的上一级，再进入 maps
    script_dir = os.path.dirname(os.path.abspath(__file__))
    target_dir = os.path.join(os.path.dirname(script_dir), "maps")
    
    if not os.path.exists(json_path):
        print(f"❌ 运行失败: 找不到输入的 JSON 文件 {json_path}")
        return

    # 2. 解析输入的 JSON
    with open(json_path, 'r') as f:
        try:
            data = json.load(f)
        except Exception as e:
            print(f"❌ JSON 格式错误: {e}")
            return

    # 获取原始图片信息
    source_dir = os.path.dirname(json_path)
    png_filename = data.get('image_path', 'map.png')
    source_png_path = os.path.join(source_dir, png_filename)
    
    if not os.path.exists(source_png_path):
        print(f"❌ 找不到原始图片: {source_png_path}")
        return

    # 3. 准备目标文件名
    map_name = os.path.splitext(png_filename)[0]
    target_yaml_path = os.path.join(target_dir, f"{map_name}.yaml")
    target_png_path = os.path.join(target_dir, png_filename)

    # 4. 执行图片拷贝 (关键步骤)
    try:
        shutil.copy2(source_png_path, target_png_path)
        print(f"✅ 图片已拷贝至: {target_png_path}")
    except Exception as e:
        print(f"❌ 图片拷贝失败: {e}")
        return

    # 5. 生成符合 Nav2 标准的 YAML
    # 注意：'image' 字段只需写文件名，因为 yaml 和 png 现在在同一个文件夹里
    nav2_config = {
        'image': png_filename, 
        'mode': 'trinary',
        'resolution': data.get('resolution', 0.05),
        'origin': [
            data.get('origin_x', 0.0),
            data.get('origin_y', 0.0),
            data.get('origin_yaw', 0.0)
        ],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25
    }

    # 6. 写入目标 YAML
    try:
        with open(target_yaml_path, 'w') as f:
            yaml.dump(nav2_config, f, default_flow_style=False)
        print(f"✅ YAML 已生成: {target_yaml_path}")
    except Exception as e:
        print(f"❌ YAML 生成失败: {e}")
        return

    print(f"\n🚀 转换完成！你现在可以运行导航并加载: {map_name}.yaml")

def main():
    if len(sys.argv) > 1:
        convert_and_move_map(sys.argv[1])
    else:
        print("������用法: ros2 run humanoid_navigation2 map_converter <JSON文件绝对路径>")

if __name__ == "__main__":
    main()