import os
from torchvision import transforms, datasets
from torch.utils.data import DataLoader, random_split

def walk_through_directory(directory):
    """
    Walk through a directory and print out the number of directories and files.

    Args:
        directory (str): The path to the directory to walk through.
    """
    for root, dirs, files in os.walk(directory):
        print(f"There are {len(dirs)} directories and {len(files)} files in '{root}'")

def get_dataloaders(data_dir, batch_size=32, train_split=0.8):
    transfrom = transforms.Compose([
        transforms.Resize((224,224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225])
    ])

    dataset = datasets.ImageFolder(root=data_dir, transform=transfrom)

    train_size = int(train_split * len(dataset))
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = random_split(dataset, [train_size, val_size])

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)
    
    class_names = dataset.classes
    return train_loader, val_loader, class_names