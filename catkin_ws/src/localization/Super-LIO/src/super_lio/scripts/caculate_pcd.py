#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Project PCD to Image with Adaptive Thresholding (No Rectangle Detection)
"""

from gc import garbage
import pcl
import numpy as np
import argparse
import cv2
import open3d as o3d
import matplotlib.pyplot as plt
from typing import Tuple

def project_pcd_to_image(xyz, x_range=(-7.1, 0.9), y_range=(-2.3, 1.3), resolution=0.01):
    """将点云投影为图像"""
    x_min, x_max = x_range
    y_min, y_max = y_range
    
    width = int((x_max - x_min) / resolution) + 1
    height = int((y_max - y_min) / resolution) + 1
    
    print(f"[INFO] Image size: {width}x{height} pixels")
    print(f"[INFO] Resolution: {resolution} meters/pixel")
    
    height_map = np.full((height, width), np.nan, dtype=np.float32)
    count_map = np.zeros((height, width), dtype=np.int32)
    
    for x, y, z in xyz:
        px = int((x - x_min) / resolution)
        py = int((y - y_min) / resolution)
        
        if 0 <= px < width and 0 <= py < height:
            if np.isnan(height_map[py, px]):
                height_map[py, px] = z
            else:
                height_map[py, px] = (height_map[py, px] + z) / 2
            count_map[py, px] += 1
    
    valid_pixels = np.sum(count_map > 0)
    print(f"[INFO] Valid pixels: {valid_pixels}/{width*height} ({valid_pixels/(width*height)*100:.1f}%)")
    
    return height_map, count_map, (x_min, y_min, x_max, y_max)

def normalize_height_map(height_map):
    """归一化高度图到0-255范围"""
    valid_heights = height_map[~np.isnan(height_map)]
    
    if len(valid_heights) == 0:
        print("[WARNING] No valid heights found")
        return np.zeros_like(height_map, dtype=np.uint8)
    
    min_h = valid_heights.min()
    max_h = valid_heights.max()
    
    if max_h - min_h < 1e-6:
        normalized = np.zeros_like(height_map, dtype=np.uint8)
    else:
        normalized = ((height_map - min_h) / (max_h - min_h) * 255).astype(np.uint8)
    
    normalized[np.isnan(height_map)] = 0
    
    print(f"[INFO] Height range: [{min_h:.3f}, {max_h:.3f}] meters")
    
    return normalized

def apply_perspective_transform(image: np.ndarray, bounds: Tuple[float, float, float, float]) -> np.ndarray:
    """
    对图像应用透视变换，校正倾斜视角
    
    Args:
        image: 输入图像（uint8格式）
        bounds: 边界信息 (x_min, y_min, x_max, y_max)
    
    Returns:
        transformed_image: 透视变换后的图像
    """
    height, width = image.shape[:2]
    
    # 定义源点（假设图像四个角）
    src_points = np.float32([
        [0, 0],           # 左上角
        [width, 0],       # 右上角
        [width, height],  # 右下角
        [0, height]       # 左下角
    ])
    
    # 定义目标点（轻微调整模拟透视校正）
    # 根据实际场景调整偏移量
    offset_top = 50
    offset_bottom = 20
    
    dst_points = np.float32([
        [offset_top, 0],                    # 左上角内移
        [width - offset_top, 0],            # 右上角内移
        [width - offset_bottom, height],    # 右下角内移较少
        [offset_bottom, height]             # 左下角内移较少
    ])
    
    # 计算透视变换矩阵
    M = cv2.getPerspectiveTransform(src_points, dst_points)
    
    # 应用透视变换
    transformed_image = cv2.warpPerspective(image, M, (width, height), flags=cv2.INTER_LINEAR)
    
    print(f"[INFO] Perspective transform applied | Offset: top={offset_top}, bottom={offset_bottom}")
    
    return transformed_image

def apply_gaussian_filter(image, kernel_size=5, sigma=1.0):
    """高斯滤波"""
    if kernel_size % 2 == 0:
        kernel_size += 1
    filtered = cv2.GaussianBlur(image, (kernel_size, kernel_size), sigma)
    print(f"[INFO] Applied Gaussian filter (kernel={kernel_size}, sigma={sigma})")
    return filtered

def apply_median_filter(image, kernel_size=3):
    """中值滤波"""
    if kernel_size % 2 == 0:
        kernel_size += 1
    filtered = cv2.medianBlur(image, kernel_size)
    print(f"[INFO] Applied Median filter (kernel={kernel_size})")
    return filtered

def detect_ridge_lines(image, sigma=1.0, threshold=0.1):
    """
    使用Hessian矩阵检测脊线
    
    Args:
        image: 输入灰度图像
        sigma: 高斯平滑的标准差
        threshold: 脊线响应阈值（0-1）
    
    Returns:
        ridge_image: 脊线检测结果图像
    """
    # 将图像转换为float32
    image_float = image.astype(np.float32) / 255.0
    
    # 使用高斯导数计算二阶导数
    dx = cv2.Sobel(image_float, cv2.CV_32F, 2, 0, ksize=3)
    dy = cv2.Sobel(image_float, cv2.CV_32F, 0, 2, ksize=3)
    dxy = cv2.Sobel(image_float, cv2.CV_32F, 1, 1, ksize=3)
    
    # 计算Hessian矩阵的特征值
    # Hessian矩阵: [dxx, dxy; dxy, dyy]
    # 特征值 = (dxx + dyy)/2 +/- sqrt(((dxx - dyy)/2)^2 + dxy^2)
    
    trace = dx + dy
    det = dx * dy - dxy * dxy
    
    # 计算较小特征值（脊线响应）
    # lambda1 <= lambda2
    sqrt_term = np.sqrt(np.maximum(0, ((dx - dy) / 2)**2 + dxy**2))
    lambda1 = (trace / 2) - sqrt_term
    
    # 脊线响应：较小特征值为负（表示脊线）
    ridge_response = -lambda1
    ridge_response[ridge_response < 0] = 0
    
    # 归一化
    max_response = ridge_response.max()
    if max_response > 0:
        ridge_response = ridge_response / max_response
    
    # 应用阈值
    ridge_image = (ridge_response > threshold).astype(np.uint8) * 255
    
    # 形态学细化
    # kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    # ridge_thinned = cv2.ximgproc.thinning(ridge_image)
    
    # print(f"[INFO] Ridge line detection completed (sigma={sigma}, threshold={threshold})")
    
    return ridge_response, ridge_image

def apply_ridge_enhancement(image, iterations=1):
    """
    使用形态学方法增强脊线
    
    Args:
        image: 输入灰度图像
        iterations: 细化迭代次数
    
    Returns:
        enhanced_image: 增强后的脊线图像
    """
    # 使用顶帽变换增强亮脊线
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    tophat = cv2.morphologyEx(image, cv2.MORPH_TOPHAT, kernel)
    
    # 自适应阈值
    thresh = cv2.adaptiveThreshold(tophat, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                   cv2.THRESH_BINARY, 11, 2)
    
    # 形态学细化
    ridge_enhanced = cv2.ximgproc.thinning(thresh)
    
    print(f"[INFO] Ridge enhancement completed")
    
    return ridge_enhanced



def save_image(image, output_path, colormap='gray'):
    """保存图像"""
    if colormap == 'gray':
        cv2.imwrite(output_path, image)
    else:
        colored = cv2.applyColorMap(image, getattr(cv2, f'COLORMAP_{colormap.upper()}'))
        cv2.imwrite(output_path, colored)
    print(f"✅ Saved: {output_path}")

def visualize_combined_image(original, filtered_gaussian, filtered_median, color_image, region_image, title="PCD Projection Comparison", save_path="combined_pcd_projection.png", true_show=True):
    """可视化组合图像（无矩形框）"""
    fig, axes = plt.subplots(2, 3, figsize=(24, 16))
    
    # 1. 原始灰度图
    im1 = axes[0, 0].imshow(original, cmap='gray', origin='lower')
    axes[0, 0].set_title('Original Grayscale Height Map')
    axes[0, 0].set_xlabel('X Pixel')
    axes[0, 0].set_ylabel('Y Pixel')
    plt.colorbar(im1, ax=axes[0, 0], label='Height (normalized)')
    
    # 2. 高斯滤波后灰度图
    im2 = axes[0, 1].imshow(filtered_gaussian, cmap='gray', origin='lower')
    axes[0, 1].set_title('Gaussian Filtered Height Map')
    axes[0, 1].set_xlabel('X Pixel')
    axes[0, 1].set_ylabel('Y Pixel')
    plt.colorbar(im2, ax=axes[0, 1], label='Height (normalized)')
    
    # 3. 中值滤波后灰度图
    im3 = axes[0, 2].imshow(filtered_median, cmap='gray', origin='lower')
    axes[0, 2].set_title('Median Filtered Height Map')
    axes[0, 2].set_xlabel('X Pixel')
    axes[0, 2].set_ylabel('Y Pixel')
    plt.colorbar(im3, ax=axes[0, 2], label='Height (normalized)')
    
    # 4. 彩色热力图
    im4 = axes[1, 0].imshow(color_image, cmap='gray', origin='lower')
    axes[1, 0].set_title('Colored Height Map (Gray Colormap)')
    axes[1, 0].set_xlabel('X Pixel')
    axes[1, 0].set_ylabel('Y Pixel')
    plt.colorbar(im4, ax=axes[1, 0], label='Height (normalized)')
    
    # 5. 自适应二值化图
    im5 = axes[1, 1].imshow(region_image, cmap='gray', origin='lower')
    axes[1, 1].set_title('Adaptive Threshold Image (32 Regions)')
    axes[1, 1].set_xlabel('X Pixel')
    axes[1, 1].set_ylabel('Y Pixel')
    plt.colorbar(im5, ax=axes[1, 1], label='Height (normalized)')
    
    # 6. 密度热力图（可选，若不需要可删除）
    # im6 = axes[1, 2].imshow(density_map, cmap='jet', origin='lower')
    # axes[1, 2].set_title('Density Heatmap')
    # axes[1, 2].set_xlabel('X Pixel')
    # axes[1, 2].set_ylabel('Y Pixel')
    # plt.colorbar(im6, ax=axes[1, 2], label='Density')
    
    plt.suptitle(title, fontsize=16)
    plt.tight_layout()
    
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"✅ Combined image saved to: {save_path}")
    
    if true_show:
        plt.show()

def draw_subregions_with_stats(gray_image, color_image, num_regions=1024, line_color=(0, 255, 0), 
                              line_thickness=1, font_scale=0.3, show_stats=True):
    """
    在图像上绘制子区域网格线，并在每个区域内显示统计信息
    
    Args:
        gray_image: 输入灰度图像
        color_image: 输入彩色图像
        num_regions: 子区域总数（默认1024，即32×32网格）
        line_color: 网格线颜色（BGR格式）
        line_thickness: 网格线粗细
        font_scale: 字体缩放因子
        show_stats: 是否显示统计信息文本
    Returns:
        image_with_grid: 带网格线和统计信息的图像
    """
    height, width = gray_image.shape[:2]
    
    # 计算每行/每列的区域数
    regions_per_row = int(np.sqrt(num_regions))
    region_height = height // regions_per_row
    region_width = width // regions_per_row
    
    print(f"[INFO] Drawing {regions_per_row}×{regions_per_row} grid with statistics")
    print(f"[INFO] Region size: {region_width}×{region_height} pixels")
    
    # 转换为彩色图像以便绘制彩色网格和文本
    
    image_with_grid = gray_image.copy()
    
    # 字体设置
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_color = (255, 0, 0)  # 蓝色文本
    
    # 遍历每个区域
    for row in range(regions_per_row):
        for col in range(regions_per_row):
            # 计算当前区域的边界
            x_start = col * region_width
            x_end = (col + 1) * region_width if col < regions_per_row - 1 else width
            y_start = row * region_height
            y_end = (row + 1) * region_height if row < regions_per_row - 1 else height
            
            # 提取当前区域
            region = gray_image[y_start:y_end, x_start:x_end]
            color_region = color_image[y_start:y_end, x_start:x_end]
            
            # 计算区域的统计信息
            region_mean = np.mean(region)
            region_std = np.std(region)
            region_max = np.max(region)
            region_min = np.min(region)
            
            print(f"[INFO] Region {row * regions_per_row + col}: "
                  f"mean={region_mean:.1f}, std={region_std:.1f}, "
                  f"max={region_max:.1f}, min={region_min:.1f}")
            
            # 只保留当前区域中大于中值的部分
            if region_mean > 0:
                color_region[region < region_mean * 1.05] = 0  # 黑色
                color_region[region >= region_mean * 1.05] = 255  # 白色
            else:
                color_region[:] = 0
                

            # 在方格内显示统计信息
            if show_stats:
                # 计算区域中心位置
                center_x = (x_start + x_end) // 2
                center_y = (y_start + y_end) // 2
                
                # 格式化统计信息文本（简化显示，避免拥挤）
                stats_text = f"{row * regions_per_row + col}"
                
                # 计算文本大小以居中显示
                (text_width, text_height), _ = cv2.getTextSize(
                    stats_text, font, font_scale, 1
                )
                
                # 调整文本位置使其居中
                text_x = center_x - text_width // 2
                text_y = center_y + text_height // 2

               
                # 绘制统计信息文本
                cv2.putText(
                    image_with_grid,
                    stats_text,
                    (text_x, text_y),
                    font,
                    font_scale,
                    text_color,
                    1,  # 文本粗细
                    cv2.LINE_AA
                )
    
    # 绘制垂直线
    for i in range(1, regions_per_row):
        x = i * region_width
        cv2.line(image_with_grid, (x, 0), (x, height), line_color, line_thickness)
    
    # 绘制水平线
    for i in range(1, regions_per_row):
        y = i * region_height
        cv2.line(image_with_grid, (0, y), (width, y), line_color, line_thickness)
    
    return image_with_grid

def apply_mls_smoothing_pcl(xyz, search_radius=0.1):
    """
    使用PCL的移动最小二乘法(MLS)平滑点云
    
    Args:
        xyz: Nx3点云数组
        search_radius: 搜索半径
    Returns:
        平滑后的点云数组
    """
    
    # 创建PCL点云对象
    cloud = pcl.PointCloud()
    cloud.from_array(xyz.astype(np.float32))
    
    # 检查是否有足够的点
    if cloud.size < 10:
        print("[WARNING] Too few points for MLS smoothing")
        return xyz
        
    # 创建MLS平滑对象
    mls = cloud.make_moving_least_squares()
    
    # 设置参数
    mls.set_search_radius(search_radius)
    mls.set_polynomial_order(2)  # 二次多项式拟合
    mls.set_point_density(0)      # 不增加新点
    
    # 执行平滑
    smoothed_cloud = mls.process()
    
    # 转换回numpy数组
    smoothed_xyz = smoothed_cloud.to_array()
    
    print(f"[INFO] PCL MLS smoothing applied (search_radius={search_radius})")
    print(f"[INFO] Points before: {len(xyz)}, after: {len(smoothed_xyz)}")
    
    return smoothed_xyz

def filter_intensity_region(pcd_path, intensity_thresh=830, resolution=0.01, 
                          gaussian_kernel=5, gaussian_sigma=1.0, median_kernel=3,
                          num_regions=512):
    # ========== 1. 读取 PCD ==========
    cloud = pcl.load_XYZI(pcd_path)
    print(f"[INFO] Loaded {cloud.size} points")

    # 进行均匀滤波
    vox = cloud.make_voxel_grid_filter()
    leaf_size = 2*resolution
    
    # 设置体素大小
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    
    # 执行滤波
    filtered_cloud = vox.filter()
    
    print(f"[INFO] PCL体素滤波完成 | 原始点数: {cloud.size} | 滤波后点数: {filtered_cloud.size}")
    print(f"[INFO] 体素大小: {leaf_size}m")

    # ========== 2. NumPy 解析 ==========
    data = filtered_cloud.to_array()
    xyz = data[:, :3]
    intensity = data[:, 3]

    # ========== 3. 强度过滤 ==========
    # mask = intensity > intensity_thresh
    # xyz_f = xyz[mask]
    # print(f"[INFO] After intensity filter: {xyz_f.shape[0]} points")
    xyz_f = xyz
    
    
    # ========== 4. 投影到图像 ==========
    print("\n" + "=" * 60)
    print("Projecting PCD to Image...")
    print("=" * 60)
    
    # 目标点云模板面全局尺寸范围
    x_range = (-6.3, -3.5)
    y_range = (-2.1, 1.1)
    
    height_map, count_map, bounds = project_pcd_to_image(
        xyz_f,
        x_range=x_range,
        y_range=y_range,
        resolution=resolution
    )

    
    # ========== 5. 归一化和滤波 ==========
    normalized_depth = normalize_height_map(height_map)
    # median_filtered = apply_median_filter(normalized_depth, median_kernel)

     #进行透视变化
    normalized_depth_perspective = apply_perspective_transform(normalized_depth, bounds)

    # 用 Sobel 计算 X / Y 方向梯度，再取梯度幅值并归一化
    gradient_x = cv2.Sobel(normalized_depth_perspective, cv2.CV_64F, 1, 0, ksize=3)
    gradient_y = cv2.Sobel(normalized_depth_perspective, cv2.CV_64F, 0, 1, ksize=3)
    gradient_magnitude = np.sqrt(np.mean(gradient_x**2 + gradient_y**2))
    
    print("\n" + "=" * 60)
    print("Applying Ridge Line Detection...")
    print("=" * 60)
    
    # ========== 6. 脊线提取处理 ==========
    # ridge_response, ridge_enhanced = detect_ridge_lines(median_filtered, sigma=1.0, threshold=resolution/5)
    # ridge_enhanced = apply_ridge_enhancement(ridge_image)
    # ridge_enhanced = apply_gaussian_filter(ridge_enhanced, gaussian_kernel, gaussian_sigma)
    
    print(f"[INFO] Ridge detection completed")
    
    # ========== 7. 自适应二值化 ==========
    print("\n" + "=" * 60)
    print("Applying Adaptive Thresholding...")
    print("=" * 60)
    
    color_image_perspective = cv2.applyColorMap(normalized_depth_perspective, cv2.COLORMAP_JET)
    color_image = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)
    

    # 将median_filtered划分为256个子区域并可视化
    # median_with_subregions = draw_subregions_with_stats(
    #     median_filtered,
    #     color_image, 
    #     num_regions=num_regions,
    #     line_color=(255, 0, 0),  # 绿色网格线
    #     line_thickness=1
    # )
    
    # visualize_combined_image(
    #     normalized_depth,
    #     ridge_enhanced,
    #     median_filtered,    
    #     color_image,
    #     median_with_subregions,
    #     title="PCD Projection with Ridge Detection",
    #     save_path="combined_pcd_projection.png",
    #     true_show=False
    # )
    
    save_image(color_image_perspective, "normalized_depth_perspective.png")
    save_image(gradient_magnitude, "gradient_magnitude.png")
    
    # save_image(ridge_enhanced, "ridge_enhanced.png")
    print(f"[INFO] Ridge images saved")
    
    # 保存高度数据
    # np.save("height_map.npy", height_map)
    # print(f"[INFO] Height data saved to: height_map.npy")
    
    return color_image

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--thresh", type=float, default=800)
    parser.add_argument("--resolution", type=float, default=0.05, 
                       help="Image resolution in meters per pixel")
    parser.add_argument("--gaussian-kernel", type=int, default=5,
                       help="Gaussian kernel size (must be odd)")
    parser.add_argument("--gaussian-sigma", type=float, default=1.0,
                       help="Gaussian kernel standard deviation")
    parser.add_argument("--median-kernel", type=int, default=3,
                       help="Median filter kernel size (must be odd)")
    
    # 仅保留自适应二值化参数
    parser.add_argument("--num-regions", type=int, default=1024,
                       help="Number of regions for adaptive thresholding (default: 1024 for 32*32 grids)")
    
    parser.add_argument("--no-display", action="store_true",
                       help="Do not display images")
    args = parser.parse_args()

    result = filter_intensity_region(
        args.pcd,
        args.thresh,
        args.resolution,
        args.gaussian_kernel,
        args.gaussian_sigma,
        args.median_kernel,
        args.num_regions
    )
    
    if args.no_display:
        print("[INFO] Skipping display as requested")