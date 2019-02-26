import torch
from dataloader import DataReader
from torch.utils.data import Dataset, DataLoader

import os, sys
import numpy as np


class PointCloudDataset(Dataset):

    def __init__(self, txt_dir):
        """
        Args:
            txt_dir (string): Directory with all the txt files.
            transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        self.data_path = txt_dir
        self.files = os.listdir(txt_dir)
        #print(self.files)


    def __len__(self):
        return len(self.files)

    def __getitem__(self, idx):
        pc_name = os.path.join(self.data_path,
                                self.files[idx])

        dr = DataReader()
        points = dr.getSam(pc_name)
        label = self.files[idx].split('_')
        label = np.array(int(label[0])-1, dtype=np.int64)
        sample = {'points': points, 'label': label}
        sample = self.to_tensor(sample)
        return sample['points'], sample['label']

    def to_tensor(self, sample):
        points, label = sample['points'], sample['label']

        # swap color axis because
        # numpy input: H x W x D x C
        # torch conv3d input: (N,C,D,H,W) , N:num of samples
        # torch input: C x D x H x W
        points = points.transpose((3, 2, 0, 1))
        return {'points': torch.from_numpy(points),
            'label': torch.from_numpy(label)}

"""
pcd = PointCloudDataset("/home/ophelia/data/txt_cleansed")
print(pcd.__len__())
a = pcd.__getitem__(1)
"""
