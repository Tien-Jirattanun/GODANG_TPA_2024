import cv2
import os

def extract_frames(video_path, output_folder, fps=10):
    cap = cv2.VideoCapture(video_path)
    video_fps = cap.get(cv2.CAP_PROP_FPS)
    
    frame_interval = int(round(video_fps / fps))
    
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    frame_count = 0
    
    while(cap.isOpened()):
        ret, frame = cap.read()
        
        if not ret:
            break
        
        if frame_count % frame_interval == 0:
            frame_name = os.path.join(output_folder, f"framecompet_silo_{frame_count // frame_interval:04d}.jpg")
            cv2.imwrite(frame_name, frame)
            print(f"Saved {frame_name}")
        
        frame_count += 1
    
    cap.release()
    cv2.destroyAllWindows()

# Example usage
video_path = "./video_compet/Screen Recording 2567-05-11 at 15.13.50.mp4"
output_folder = "./ML_ball/silo_dataset"
extract_frames(video_path, output_folder)
