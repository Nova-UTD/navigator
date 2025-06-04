"""
Package:   path_planners
Filename:  neuralnet.py
Author:    Bennett Daniel

Neural Network Path Planner - Training Module

Description:
- This module implements a neural network-based path planner using a U-Net architecture.
- Inputs: costmap, start position, and goal position 
- Generates a probability map of the optimal path. 
- The model is trained on either synthetic data generated with A* algorithm or real-world path planning data.

How to Train the Model:
1. Prepare your data:
   - For synthetic data: No preparation needed, use --use_synthetic flag
   - For real data: Prepare costmaps, paths, and a CSV file with start/goal coordinates like dataset.csv

2. Run the training script with appropriate parameters:
   - Adjust batch size, learning rate, and epochs as needed
   - Select appropriate device (CPU, CUDA for NVIDIA GPUs, MPS for Apple Silicon)
   - Specify paths to your ground truth data files if using real data

Example Command:
# Train with synthetic data on CPU
python neuralnet.py --use_synthetic --epochs 50 --batch_size 16 --lr 0.001 --device cpu

# Train with real data on GPU
python neuralnet.py --csv_path /path/to/dataset.csv --costmaps_dir /path/to/costmaps --paths_dir /path/to/paths --device cuda --epochs 100 --batch_size 32

# Train on Apple Silicon GPU
python neuralnet.py --use_synthetic --device mps --epochs 75 --batch_size 24

The trained model will be saved in the 'models' directory, with the best model saved as 'best_model.pth'.
Training and validation loss curves will be saved as 'training_loss.png'.
"""

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
import os
import scipy.ndimage
from PIL import Image
import random
import argparse
import pandas as pd # Add pandas import
import torch.nn.functional as F # Ensure F is imported if not already

# --- U-Net Components ---

class DoubleConv(nn.Module):
    """(convolution => [BN] => ReLU) * 2"""

    def __init__(self, in_channels, out_channels, mid_channels=None):
        super().__init__()
        if not mid_channels:
            mid_channels = out_channels
        self.double_conv = nn.Sequential(
            nn.Conv2d(in_channels, mid_channels, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(mid_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(mid_channels, out_channels, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True)
        )

    def forward(self, x):
        return self.double_conv(x)

class Down(nn.Module):
    """Downscaling with maxpool then double conv"""

    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.maxpool_conv = nn.Sequential(
            nn.MaxPool2d(2),
            DoubleConv(in_channels, out_channels)
        )

    def forward(self, x):
        return self.maxpool_conv(x)

class Up(nn.Module):
    """Upscaling then double conv"""

    def __init__(self, in_channels, out_channels, bilinear=True):
        super().__init__()

        # if bilinear, use the normal convolutions to reduce the number of channels
        if bilinear:
            self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
            self.conv = DoubleConv(in_channels, out_channels, in_channels // 2)
        else:
            self.up = nn.ConvTranspose2d(in_channels, in_channels // 2, kernel_size=2, stride=2)
            self.conv = DoubleConv(in_channels, out_channels)

    def forward(self, x1, x2):
        x1 = self.up(x1)
        # input is CHW
        diffY = x2.size()[2] - x1.size()[2]
        diffX = x2.size()[3] - x1.size()[3]

        x1 = F.pad(x1, [diffX // 2, diffX - diffX // 2,
                        diffY // 2, diffY - diffY // 2])
        # if you have padding issues, see
        # https://github.com/HaiyongJiang/U-Net-Pytorch-Unstructured-Buggy/commit/0e854509c2cea854e247a9c615f175176f56347b
        x = torch.cat([x2, x1], dim=1)
        return self.conv(x)

class OutConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(OutConv, self).__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, kernel_size=1)

    def forward(self, x):
        return self.conv(x)

# --- U-Net Architecture ---
class UNet(nn.Module):
    def __init__(self, n_channels, n_classes, bilinear=True):
        super(UNet, self).__init__()
        self.n_channels = n_channels
        self.n_classes = n_classes
        self.bilinear = bilinear

        self.inc = DoubleConv(n_channels, 64)
        self.down1 = Down(64, 128)
        self.down2 = Down(128, 256)
        self.down3 = Down(256, 512)
        factor = 2 if bilinear else 1
        self.down4 = Down(512, 1024 // factor)
        self.up1 = Up(1024, 512 // factor, bilinear)
        self.up2 = Up(512, 256 // factor, bilinear)
        self.up3 = Up(256, 128 // factor, bilinear)
        self.up4 = Up(128, 64, bilinear)
        self.outc = OutConv(64, n_classes)

    def forward(self, x, start_pos, goal_pos):
        # Create start and goal channels
        batch_size, _, height, width = x.shape
        start_channel = torch.zeros((batch_size, 1, height, width), device=x.device, dtype=x.dtype)
        goal_channel = torch.zeros((batch_size, 1, height, width), device=x.device, dtype=x.dtype)

        # Convert positions to integers
        start_pos = start_pos.long()
        goal_pos = goal_pos.long()

        # Set start and goal positions efficiently using batch indexing
        batch_indices = torch.arange(batch_size, device=x.device)
        start_channel[batch_indices, 0, start_pos[:, 0], start_pos[:, 1]] = 1.0
        goal_channel[batch_indices, 0, goal_pos[:, 0], goal_pos[:, 1]] = 1.0

        # Concatenate input, start, and goal channels
        x_combined = torch.cat([x, start_channel, goal_channel], dim=1) # Shape: [B, 3, H, W]

        # U-Net Forward Pass
        x1 = self.inc(x_combined)
        x2 = self.down1(x1)
        x3 = self.down2(x2)
        x4 = self.down3(x3)
        x5 = self.down4(x4)
        x = self.up1(x5, x4)
        x = self.up2(x, x3)
        x = self.up3(x, x2)
        x = self.up4(x, x1)
        logits = self.outc(x)

        # Apply sigmoid activation for probability map output
        return torch.sigmoid(logits)

# Dataset for path planning
class PathPlanningDataset(Dataset):
    def __init__(self, costmaps, paths, starts, goals, transform=None):
        self.costmaps = costmaps
        self.paths = paths
        self.starts = starts
        self.goals = goals
        self.transform = transform
        
    def __len__(self):
        return len(self.costmaps)
    
    def __getitem__(self, idx):
        costmap = self.costmaps[idx]
        path = self.paths[idx]
        start = self.starts[idx]
        goal = self.goals[idx]
        
        # Convert to tensors
        costmap_tensor = torch.tensor(costmap, dtype=torch.float32).unsqueeze(0)  # Add channel dimension
        path_tensor = torch.tensor(path, dtype=torch.float32).unsqueeze(0)  # Add channel dimension
        start_tensor = torch.tensor(start, dtype=torch.float32)
        goal_tensor = torch.tensor(goal, dtype=torch.float32)
        
        if self.transform:
            costmap_tensor = self.transform(costmap_tensor)
            path_tensor = self.transform(path_tensor)
        
        return costmap_tensor, path_tensor, start_tensor, goal_tensor

# Function to generate synthetic data
def generate_synthetic_data(num_samples=1000, width=200, height=200, obstacle_density=0.2):
    costmaps = []
    paths = []
    starts = []
    goals = []
    
    for _ in range(num_samples):
        # Generate random costmap with obstacles
        costmap = np.ones((height, width)) * 100  # Free space
        
        # Add random obstacles
        num_obstacles = int(width * height * obstacle_density / 100)  # Adjust obstacle density
        for _ in range(num_obstacles):
            x = random.randint(0, width-1)
            y = random.randint(0, height-1)
            radius = random.randint(5, 15)
            
            # Create circular obstacle
            for i in range(max(0, y-radius), min(height, y+radius)):
                for j in range(max(0, x-radius), min(width, x+radius)):
                    if (i-y)**2 + (j-x)**2 <= radius**2:
                        costmap[i, j] = 0  # Obstacle
        
        # Generate random start and goal positions (ensure they're not in obstacles)
        while True:
            start_y = random.randint(10, height-10)
            start_x = random.randint(10, width-10)
            if costmap[start_y, start_x] > 50:  # Not an obstacle
                break
        
        while True:
            goal_y = random.randint(10, height-10)
            goal_x = random.randint(10, width-10)
            # Ensure goal is far enough from start
            if costmap[goal_y, goal_x] > 50 and ((goal_y-start_y)**2 + (goal_x-start_x)**2) > (width/4)**2:
                break
        
        # Generate a simple path (this would be replaced by your path planners)
        path = np.zeros((height, width))
        
        # Simple A* path finding
        from heapq import heappush, heappop
        
        def heuristic(a, b):
            return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
        
        def astar(array, start, goal):
            neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
            close_set = set()
            came_from = {}
            gscore = {start:0}
            fscore = {start:heuristic(start, goal)}
            oheap = []
            heappush(oheap, (fscore[start], start))
            
            while oheap:
                current = heappop(oheap)[1]
                
                if current == goal:
                    data = []
                    while current in came_from:
                        data.append(current)
                        current = came_from[current]
                    data.append(start)
                    data.reverse()
                    return data
                
                close_set.add(current)
                for i, j in neighbors:
                    neighbor = current[0] + i, current[1] + j
                    if 0 <= neighbor[0] < array.shape[0] and 0 <= neighbor[1] < array.shape[1]:
                        if array[neighbor[0]][neighbor[1]] < 50:  # Obstacle
                            continue
                            
                        tentative_g_score = gscore.get(current, 0) + heuristic(current, neighbor)
                        
                        if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                            continue
                            
                        if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                            came_from[neighbor] = current
                            gscore[neighbor] = tentative_g_score
                            fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                            heappush(oheap, (fscore[neighbor], neighbor))
            
            return []
        
        # Find path using A*
        path_coords = astar(costmap, (start_y, start_x), (goal_y, goal_x))
        
        if path_coords:
            for y, x in path_coords:
                path[y, x] = 1.0
                
            # Dilate the path slightly to make it more visible
            path = scipy.ndimage.binary_dilation(path, iterations=2).astype(np.float32)
            
            # Normalize costmap
            costmap = costmap / 100.0
            
            costmaps.append(costmap)
            paths.append(path)
            starts.append((start_y, start_x))
            goals.append((goal_y, goal_x))
    
    return costmaps, paths, starts, goals

# Function to load data from files
def load_real_data(csv_path, costmaps_dir, paths_dir, target_height=200, target_width=200, dilate_path=True, dilation_iterations=2):
    costmaps = []
    paths = []
    starts = []
    goals = []

    df = pd.read_csv(csv_path)

    for index, row in df.iterrows():
        costmap_filename = row['costmap'].replace('.png', '.npy')
        # Get start/goal coordinates
        start_col, start_row = int(row['start_x']), int(row['start_y'])
        goal_col, goal_row = int(row['end_x']), int(row['end_y'])

        # Construct file paths
        costmap_path = os.path.join(costmaps_dir, costmap_filename)
        path_filename = os.path.splitext(row['costmap'])[0] + '.npy'
        path_filepath = os.path.join(paths_dir, path_filename)

        # Skip if files don't exist
        if not os.path.exists(costmap_path) or not os.path.exists(path_filepath):
            continue

        try:
            # Load costmap directly from numpy file
            costmap = np.load(costmap_path)
            
            # Normalize costmap to [0, 1] range if needed
            if costmap.max() > 1.0:
                costmap = costmap / 255.0
                
            # Resize if needed
            if costmap.shape[0] != target_height or costmap.shape[1] != target_width:
                # Convert to PIL Image for resizing
                costmap_img = Image.fromarray((costmap * 255).astype(np.uint8))
                costmap_img = costmap_img.resize((target_width, target_height), Image.Resampling.BILINEAR)
                costmap = np.array(costmap_img, dtype=np.float32) / 255.0

            # Load path from numpy file
            path_data = np.load(path_filepath)
            
            # Create an empty path map
            path_map = np.zeros((target_height, target_width), dtype=np.float32)
            
            # If path_data is a list of coordinates
            if len(path_data.shape) == 2 and path_data.shape[1] == 2:
                # Scale coordinates if needed
                if costmap.shape[0] != target_height or costmap.shape[1] != target_width:
                    scale_y = target_height / costmap.shape[0]
                    scale_x = target_width / costmap.shape[1]
                    path_data[:, 0] = np.clip(path_data[:, 0] * scale_y, 0, target_height-1)
                    path_data[:, 1] = np.clip(path_data[:, 1] * scale_x, 0, target_width-1)
                
                # Convert to integers
                path_data = path_data.astype(np.int32)
                
                # Plot path on the map
                for y, x in path_data:
                    if 0 <= y < target_height and 0 <= x < target_width:
                        path_map[y, x] = 1.0
                
                # Dilate the path to make it more visible
                if dilate_path:
                    path_map = scipy.ndimage.binary_dilation(
                        path_map, iterations=dilation_iterations
                    ).astype(np.float32)
            else:
                # If path_data is already a map, resize it if needed
                if path_data.shape[0] != target_height or path_data.shape[1] != target_width:
                    path_img = Image.fromarray((path_data * 255).astype(np.uint8))
                    path_img = path_img.resize((target_width, target_height), Image.Resampling.BILINEAR)
                    path_map = np.array(path_img, dtype=np.float32) / 255.0
                else:
                    path_map = path_data.astype(np.float32)
                    if path_map.max() > 1.0:
                        path_map = path_map / 255.0

            # Scale start/goal coordinates if needed
            if costmap.shape[0] != target_height or costmap.shape[1] != target_width:
                start_row = int(start_row * target_height / costmap.shape[0])
                start_col = int(start_col * target_width / costmap.shape[1])
                goal_row = int(goal_row * target_height / costmap.shape[0])
                goal_col = int(goal_col * target_width / costmap.shape[1])
            
            # Ensure coordinates are within bounds
            start_row = min(max(start_row, 0), target_height-1)
            start_col = min(max(start_col, 0), target_width-1)
            goal_row = min(max(goal_row, 0), target_height-1)
            goal_col = min(max(goal_col, 0), target_width-1)

            costmaps.append(costmap)
            paths.append(path_map)
            starts.append((start_row, start_col))
            goals.append((goal_row, goal_col))
            
        except Exception as e:
            print(f"Error processing {costmap_filename}: {e}")
            continue

    return costmaps, paths, starts, goals

def train_model(model, train_loader, val_loader, device, num_epochs=50, learning_rate=0.001):
    criterion = nn.BCELoss()
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    
    # For tracking metrics
    train_losses = []
    val_losses = []
    best_val_loss = float('inf')
    
    for epoch in range(num_epochs):
        # Training phase
        model.train()
        running_loss = 0.0
        
        for costmaps, paths, starts, goals in train_loader:
            costmaps = costmaps.to(device)
            paths = paths.to(device)
            starts = starts.to(device)
            goals = goals.to(device)
            
            optimizer.zero_grad()
            
            outputs = model(costmaps, starts, goals)
            loss = criterion(outputs, paths)
            
            loss.backward()
            optimizer.step()
            
            running_loss += loss.item() * costmaps.size(0)
        
        epoch_train_loss = running_loss / len(train_loader.dataset)
        train_losses.append(epoch_train_loss)
        
        # Validation phase
        model.eval()
        running_val_loss = 0.0
        
        with torch.no_grad():
            for costmaps, paths, starts, goals in val_loader:
                costmaps = costmaps.to(device)
                paths = paths.to(device)
                starts = starts.to(device)
                goals = goals.to(device)
                
                outputs = model(costmaps, starts, goals)
                loss = criterion(outputs, paths)
                
                running_val_loss += loss.item() * costmaps.size(0)
        
        epoch_val_loss = running_val_loss / len(val_loader.dataset)
        val_losses.append(epoch_val_loss)
        
        print(f'Epoch {epoch+1}/{num_epochs}, Train Loss: {epoch_train_loss:.4f}, Val Loss: {epoch_val_loss:.4f}')
        
        # Save best model
        if epoch_val_loss < best_val_loss:
            best_val_loss = epoch_val_loss
            torch.save(model.state_dict(), 'models/best_model.pth')
            print(f'Model saved at epoch {epoch+1} with validation loss: {best_val_loss:.4f}')
    
    # Plot training and validation loss
    plt.figure(figsize=(10, 5))
    plt.plot(train_losses, label='Training Loss')
    plt.plot(val_losses, label='Validation Loss')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.title('Training and Validation Loss')
    plt.legend()
    plt.savefig('models/training_loss.png')
    plt.close()
    
    return train_losses, val_losses

def main():
    parser = argparse.ArgumentParser(description='Train Neural Network Path Planner')
    parser.add_argument('--batch_size', type=int, default=16, help='Batch size for training')
    parser.add_argument('--epochs', type=int, default=50, help='Number of epochs to train')
    parser.add_argument('--lr', type=float, default=0.001, help='Learning rate')
    parser.add_argument('--use_synthetic', action='store_true', help='Use synthetic data instead of real data')
    parser.add_argument('--csv_path', type=str, default=os.path.join(os.path.dirname(__file__), 'dataset.csv'), 
                        help='Path to CSV file with start/goal coordinates')
    parser.add_argument('--costmaps_dir', type=str, default=os.path.join(os.path.dirname(__file__), 'costmaps'), 
                        help='Directory containing costmap files')
    parser.add_argument('--paths_dir', type=str, default='path/to/paths', 
                        help='Directory containing ground truth path files')
    parser.add_argument('--device', type=str, default='cpu', choices=['cpu', 'mps', 'cuda'], 
                        help='Device to use for training (cpu, mps for M1/M2 GPU, or cuda for NVIDIA GPU)')
    args = parser.parse_args()
    
    # Create models directory if it doesn't exist
    os.makedirs('models', exist_ok=True)
    
    # Set device based on argument and availability
    if args.device == 'mps' and torch.backends.mps.is_available():
        device = torch.device("mps")
        print(f"Using MPS device (Apple GPU)")
    elif args.device == 'cuda' and torch.cuda.is_available():
        device = torch.device("cuda")
        print(f"Using CUDA device: {torch.cuda.get_device_name(0)}")
    else:
        device = torch.device("cpu")
        print("Using CPU")
    
    if args.use_synthetic:
        # Generate synthetic data
        costmaps, paths, starts, goals = generate_synthetic_data(num_samples=1000)
    else:
        # Load real data
        costmaps, paths, starts, goals = load_real_data(
            args.csv_path, args.costmaps_dir, args.paths_dir
        )
        
        if len(costmaps) == 0:
            print("No valid data found. Please check your file paths.")
            return
        
        print(f"Loaded {len(costmaps)} samples from real data.")
    
    # Split data into training and validation sets (80/20)
    split_idx = int(0.8 * len(costmaps))
    indices = list(range(len(costmaps)))
    random.shuffle(indices)
    
    train_indices = indices[:split_idx]
    val_indices = indices[split_idx:]
    
    train_costmaps = [costmaps[i] for i in train_indices]
    train_paths = [paths[i] for i in train_indices]
    train_starts = [starts[i] for i in train_indices]
    train_goals = [goals[i] for i in train_indices]
    
    val_costmaps = [costmaps[i] for i in val_indices]
    val_paths = [paths[i] for i in val_indices]
    val_starts = [starts[i] for i in val_indices]
    val_goals = [goals[i] for i in val_indices]
    
    # Create datasets and dataloaders
    train_dataset = PathPlanningDataset(train_costmaps, train_paths, train_starts, train_goals)
    val_dataset = PathPlanningDataset(val_costmaps, val_paths, val_starts, val_goals)
    
    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size)
    
    # Initialize model with 3 input channels (costmap + start + goal)
    model = UNet(n_channels=3, n_classes=1).to(device)
    
    # Train model
    train_losses, val_losses = train_model(
        model, train_loader, val_loader, device, 
        num_epochs=args.epochs, learning_rate=args.lr
    )
    
    print("Training complete!")

if __name__ == "__main__":
    main()