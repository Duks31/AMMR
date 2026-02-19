import os
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
from torchvision import transforms, datasets
from torch.utils.data import DataLoader, random_split
from pathlib import Path
from typing import Dict, List

def walk_through_directory(directory):
    """
    Walk through a directory and print out the number of directories and files.

    Args:
        directory (str): The path to the directory to walk through.
    """
    for root, dirs, files in os.walk(directory):
        print(f"There are {len(dirs)} directories and {len(files)} files in '{root}'")


def get_dataloaders(
    train_data_dir: Path,
    test_data_dir: Path,
    batch_size=32,
    num_workers=2,
):
    """
    Create train and validation dataloaders with appropriate transforms.

    Args:
        data_dir: Path to data directory
        batch_size: Batch size for dataloaders
        train_split: Proportion of data for training (0-1)
        num_workers: Number of worker processes for data loading

    Returns:
        train_loader, val_loader, class_names, dataset_size
    """

    train_transfrom = transforms.Compose(
        [
            transforms.Resize((256, 256)),
            transforms.RandomResizedCrop(224),
            transforms.RandomHorizontalFlip(p=0.5),
            transforms.RandomRotation(15),
            transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )

    val_transform = transforms.Compose(
        [
            transforms.Resize((256, 256)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )

    torch.manual_seed(42)
    train_dataset = datasets.ImageFolder(root = train_data_dir, transform=train_transfrom)
    val_dataset = datasets.ImageFolder(root = test_data_dir, transform=val_transform)

    train_loader = DataLoader(
        train_dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=num_workers,
        pin_memory=True,
    )

    val_loader = DataLoader(
        val_dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        pin_memory=True,
    )

    class_names = train_dataset.classes
    total_size = len(train_dataset) + len(val_dataset)

    print(f"Dataset loaded:")
    print(f"  Total images: {total_size}")
    print(f"  Training images: {len(train_dataset)}")
    print(f"  Validation images: {len(val_dataset)}")
    print(f"  Classes: {class_names} -> {train_dataset.class_to_idx}")

    return train_loader, val_loader, class_names, total_size


def plot_training_history(histories: List[Dict], model_names: List[str]):
    """
    Plot training and validation loss/accuracy curves.

    Args:
        histories (list[dict] or dict): One or more training histories
        model_name (str): Name of the model 
    """

    if isinstance(histories, dict):
        histories = [histories]

    if model_names is None:
        model_names = [f"Model {i+1}" for i in range(len(histories))]

    plt.figure(figsize=(14, 6))

    # Plot Loss
    plt.subplot(1, 2, 1)
    for history, name in zip(histories, model_names):
        plt.plot(history['train_loss'], label=f'{name} Train')
        plt.plot(history['val_loss'], linestyle='--', label=f'{name} Val')
    plt.title('Training and Validation Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)

    # Plot Accuracy
    plt.subplot(1, 2, 2)
    for history, name in zip(histories, model_names):
        plt.plot(history['train_acc'], label=f'{name} Train')
        plt.plot(history['val_acc'], linestyle='--', label=f'{name} Val')
    plt.title('Training and Validation Accuracy')
    plt.xlabel('Epoch')
    plt.ylabel('Accuracy (%)')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    plt.show()