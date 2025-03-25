import cv2
import os

def images_to_video(image_folder, video_name, frame_rate):
    # 获取文件夹内所有图片的文件名，并按顺序排序
    images = [img for img in os.listdir(image_folder) if img.endswith(".jpg") or img.endswith(".png")]
    images.sort()
    
    # 确定图片的宽高
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    # 定义视频的编解码器和输出文件
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 或者使用 'XVID'，'MJPG' 等
    video = cv2.VideoWriter(video_name, fourcc, frame_rate, (width, height))

    for image in images:
        img_path = os.path.join(image_folder, image)
        frame = cv2.imread(img_path)
        video.write(frame)

    video.release()

# 使用示例
image_folder = '/home/hjyeee/Data/Dataset/rpf_dataset/2024-07-04-21-12-25/zed_images'  # 替换为实际的图片文件夹路径
video_name = '/home/hjyeee/Data/Dataset/rpf_dataset/2024-07-04-21-12-25/zed_images_raw.mp4'  # 输出视频的文件名
frame_rate = 10  # 帧率（每秒显示的图片数）
images_to_video(image_folder, video_name, frame_rate)
