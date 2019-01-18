import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import math
from PointCloudDataset import PointCloudDataset

# Device configuration
device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

pcd_dataset = PointCloudDataset("/home/ophelia/data/txt_cleansed")

train_loader = torch.utils.data.DataLoader(dataset=pcd_dataset,
                                           batch_size=16,shuffle=True)
torch.set_default_tensor_type('torch.DoubleTensor')
# Hyper parameters
num_epochs = 5
num_classes = 10
batch_size = 100
learning_rate = 0.001

# 3D Convolutional neural network (two convolutional layers)
class ConvNet3d(nn.Module):
	def __init__(self, learning_rate=0.001, num_classes=26, batch_size=16, epochs=8):
		super(ConvNet3d, self).__init__()

		self.num_classes = num_classes
		self.learning_rate = learning_rate
		self.batch_size = batch_size
		self.epochs = epochs

		self.layer1 = nn.Sequential(
		    nn.Conv3d(3, 16, kernel_size=5, stride=1, padding=2),
		    nn.BatchNorm3d(16),
		    nn.ReLU(),
		    nn.MaxPool3d(kernel_size=2, stride=2))
		self.layer2 = nn.Sequential(
		    nn.Conv3d(16, 32, kernel_size=3, stride=1, padding=1),
		    nn.BatchNorm3d(32),
		    nn.ReLU(),
		    nn.MaxPool3d(kernel_size=2, stride=2))
		self.fc = nn.Linear(10*10*10*32, num_classes)

	def forward(self, x):
		out = self.layer1(x)
		out = self.layer2(out)
		out = out.reshape(out.size(0), -1)
		out = self.fc(out)
		#out = nn.Softmax(out)

		return out

model = ConvNet3d(num_classes).to(device)

# Loss and optimizer
criterion = nn.CrossEntropyLoss()
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

# Train the model
total_step = len(train_loader)
for epoch in range(num_epochs):
    for i, (pointclouds, labels) in enumerate(train_loader):

        pointclouds = pointclouds.to(device)
        labels = labels.to(device)

        # Forward pass
        outputs = model(pointclouds)
        loss = criterion(outputs, labels)

        # Backward and optimize
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if (i+1) % 2 == 0:
            print ('Epoch [{}/{}], Step [{}/{}], Loss: {:.4f}'
                   .format(epoch+1, num_epochs, i+1, total_step, loss.item()))

"""
# Test the model
model.eval()  # eval mode (batchnorm uses moving mean/variance instead of mini-batch mean/variance)
with torch.no_grad():
    correct = 0
    total = 0
    for images, labels in test_loader:
        images = images.to(device)
        labels = labels.to(device)
        outputs = model(images)
        _, predicted = torch.max(outputs.data, 1)
        total += labels.size(0)
        correct += (predicted == labels).sum().item()

    print('Test Accuracy of the model on the 10000 test images: {} %'.format(100 * correct / total))

# Save the model checkpoint
torch.save(model.state_dict(), 'model.ckpt')
"""
