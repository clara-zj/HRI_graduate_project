#!/usr/bin/env python3

import cv2
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
from torchvision import transforms
from torch.utils.data import DataLoader, Dataset
from datetime import datetime
# import pygame
import threading
from pathlib import Path
import time
import yaml
# import rclpy
# from rclpy.node import Node

# from cabo_bot.cabo_vision_source import DataSource, RealSensePipelineSource, WebcamSource, RealSenseROSTopicSource
from cabo_bot.cabo_constants import FRAME_WIDTH, FRAME_HEIGHT


EPOCHS = 6
IMAGE_COUNT = 60


class SimpleCNN(nn.Module):
    def __init__(self):
        super(SimpleCNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, kernel_size=3, padding=1)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)
        self.fc1 = nn.Linear(64 * 32 * 32, 128)
        self.fc2 = nn.Linear(128, 2)  # classify to two classes: 'target' and 'others'

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = x.view(-1, 64 * 32 * 32)
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x


class CustomDataset(Dataset):
    def __init__(self, images, labels, transform=None):
        self.images = images
        self.labels = labels
        self.transform = transform

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        image = self.images[idx]
        label = self.labels[idx]
        if self.transform:
            image = self.transform(image)
        return image, label
    


########################################################################
class PersonDetectionSystem:
    def __init__(self, data_source):
        self.data_source = data_source
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((128, 128)),
            transforms.ToTensor()
        ])
        
        self.yolo = torch.hub.load('ultralytics/yolov5', 'yolov5n')
        self.yolo.eval()
        # self.yolo = YoloModel(Path('/home/clara/cabo_ros2/yolov5n.pt'))

        self.target_images = []
        self.other_images = []
        self.cnn = SimpleCNN()
        self.criterion = nn.CrossEntropyLoss()
        self.optimizer = optim.Adam(self.cnn.parameters(), lr=0.001)

        self.thread = None
        self.running = True
        self.lock = threading.Lock()
        self.show_frame = None

    def init_vision_system(self, num_target_images=IMAGE_COUNT, epochs=EPOCHS, batch_size=4):
        if self.__collect_training_images(num_target_images):
            self.__train_cnn(epochs, batch_size)
            return True
        return False


    def __collect_training_images(self, num_target_images):
        print("Collecting images for training...")
        self.target_images = []
        self.other_images = []
        self.cnn = SimpleCNN()
        self.criterion = nn.CrossEntropyLoss()
        self.optimizer = optim.Adam(self.cnn.parameters(), lr=0.001)
        
        while len(self.target_images) < num_target_images:
            frame = None
            time.sleep(0.01)
            current_time = datetime.now()
            formatted_time = current_time.strftime('%Y-%m-%d %H:%M:%S')
            print(f"__collect_training_images Timestamp: {formatted_time}")
            frame, _ = self.data_source.get_frame()  
            if frame is None:
                print(f"Cannot capture rgb frame from {self.data_source.__class__.__name__}. Exiting...")
                time.sleep(0.01)
                continue

            with torch.no_grad():
                results = self.yolo(frame.copy())

                for *xyxy, conf, cls in results.xyxy[0]:
                    if int(cls) == 0:
                        x1, y1, x2, y2 = map(int, xyxy)
                        bbox = frame[y1:y2, x1:x2].copy()
                        self.target_images.append(bbox)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 5)

                        other_area = frame[:y1, :x1].copy()
                        if other_area.size > 0:
                            self.other_images.append(other_area)
                        
                        break

                self.show_frame(frame.copy())
        self.show_frame(np.zeros((2, 2, 3), dtype=np.uint8), '0')

        print('Data collection completed.')
        return True


    def __train_cnn(self, epochs, batch_size):
        X = self.target_images + self.other_images
        y = [1] * len(self.target_images) + [0] * len(self.other_images)
        dataset = CustomDataset(X, y, transform=self.transform)
        dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

        print("Training CNN...")
        self.cnn.train()
        for epoch in range(epochs):
            running_loss = 0.0
            for inputs, labels in dataloader:
                self.optimizer.zero_grad()
                outputs = self.cnn(inputs)
                loss = self.criterion(outputs, labels)
                loss.backward()
                self.optimizer.step()
                running_loss += loss.item()
            print(f'Epoch {epoch + 1}, Loss: {running_loss / len(dataloader)}')


    def detect_and_update(self):
        self.cnn.eval()
        frame_count = 0

        while True:
            rgb_frame, depth_frame = None, None
            time.sleep(0.01)
            with self.lock: 
                if not self.running:
                    self.on_target_detected(0, 0)
                    break

            rgb_frame, depth_frame = self.data_source.get_frame()
            if rgb_frame is None:
                print(f"Cannot capture rgb frame from {self.data_source.__class__.__name__}. Exiting...")
                self.on_target_detected(0, 0)
                self.show_frame(np.zeros((2, 2, 3), dtype=np.uint8), 
                                '0')
                continue
       
            with torch.no_grad():
                results = self.yolo(rgb_frame.copy())

            best_proba = 0
            best_bbox = None

            for *xyxy, conf, cls in results.xyxy[0]:
                if int(cls) == 0:
                    x1, y1, x2, y2 = map(int, xyxy)
                    bbox = rgb_frame[y1:y2, x1:x2]
                    img = self.transform(bbox).unsqueeze(0)
                    outputs = self.cnn(img)
                    proba = torch.softmax(outputs, dim=1)[0][1].item()
                    if proba > best_proba and proba > 0.5:
                        best_proba = proba
                        best_bbox = (x1, y1, x2, y2)

            turn_left = 0
            forwarding = 0
            if best_bbox is not None:
                # turning derection
                x1, y1, x2, y2 = best_bbox
                bbox_center_x = (x1 + x2) // 2
                frame_center_x = FRAME_WIDTH // 2
                offset_x = bbox_center_x - frame_center_x
                if (offset_x / FRAME_WIDTH) > 0.1:
                    turn_left = -1
                elif (offset_x / FRAME_WIDTH) < -0.1:
                    turn_left = 1
                else:
                    turn_left = 0

                # forwarding
                if depth_frame is not None:
                    # depth_bbox = depth_frame[y1:y2, x1:x2]
                    # closest_point_depth = np.min(depth_bbox[depth_bbox > 0]) if np.any(depth_bbox > 0) else None
                    # if closest_point_depth is not None:
                    #     print(f"Closest point depth: {closest_point_depth}")
                    #     if closest_point_depth > 1500:
                    #         forwarding = 1

                    closest_full_depth = np.min(depth_frame[depth_frame > 0]) if np.any(depth_frame > 0) else None
                    if closest_full_depth is not None:
                        print(f"Closest full depth: {closest_full_depth}")
                        if closest_full_depth > 1500:
                            forwarding = 1
                else:
                    frame_area = FRAME_WIDTH * FRAME_HEIGHT
                    bbox_percentage = ((x2 - x1) * (y2 - y1) / frame_area) * 100
                    if bbox_percentage < 40:
                        forwarding = 1
                    else:
                        forwarding = 0

                cv2.rectangle(rgb_frame, (x1, y1), (x2, y2), (0, 255, 0) , 3)
                cv2.putText(rgb_frame, 'target', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0) , 2)

                if False:
                    frame_count += 1
                    if frame_count % 20 == 0:
                        new_target_image = rgb_frame[y1:y2, x1:x2].copy()
                        new_target_images = [new_target_image].copy()
                        new_labels = [1]

                        new_dataset = CustomDataset(new_target_images, new_labels, transform=self.transform)
                        new_dataloader = DataLoader(new_dataset, batch_size=1, shuffle=True)

                        self.cnn.train()
                        # for _ in range(EPOCHS):
                        for _ in range(1):
                            running_loss = 0.0
                            for inputs, labels in new_dataloader:
                                self.optimizer.zero_grad()
                                outputs = self.cnn(inputs)
                                loss = self.criterion(outputs, labels)
                                loss.backward()
                                self.optimizer.step()
                                running_loss += loss.item()
                            print(f'Updated model - Loss: {running_loss / len(new_dataloader)}')
                        self.cnn.eval()
                
                self.on_target_detected(turn_left, forwarding)
                self.show_frame(rgb_frame.copy())

    def on_target_detected(self, turn_left, forwarding):
        pass


    def live(self):
        self.detect_and_update()

    
    def start(self):
        print('Starting detection thread...')
        with self.lock:
            self.running = True
        self.thread = threading.Thread(target=self.detect_and_update, daemon=True)
        self.thread.start()

    def stop(self):
        print('Stopping detection thread...')
        with self.lock:
            self.running = False


# if __name__ == "__main__":
#     mode = input("Select mode (1: Realsense Pipeline, 2: Webcam, 3: ROS2): ")
#     if mode == "1":
#         data_source = RealSensePipelineSource()
#     elif mode == "2":
#         data_source = WebcamSource()
#     elif mode == "3":
#         rclpy.init(args=None)
#         data_source = RealSenseROSTopicSource()
#         rclpy.spin(data_source)  # 确保ROS2节点持续运行
#     else:
#         raise ValueError("Invalid mode selected")

#     system = PersonDetectionSystem(data_source)
#     try:
#         system.start()
#     finally:
#         if mode == "3":
#             data_source.destroy_node()
#             rclpy.shutdown()

