import torch
import torch.nn as nn
import time
import copy
from torch.utils.data import DataLoader
from typing import Optional, Dict
from pathlib import Path

def train_model(model: nn.Module, train_loader: DataLoader,
                 val_loader: DataLoader, loss: nn.Module, optimizer: torch.optim.Optimizer, device: torch.device, num_epochs: int = 25, save_path: Optional[Path] = None) -> Dict:
    """
    Train a model with validation.
    
    Args:
        model: The neural network model
        train_loader: DataLoader for training data
        val_loader: DataLoader for validation data
        criterion: Loss function
        optimizer: Optimizer
        num_epochs: Number of training epochs
        device: Device to train on (cuda/cpu)
        scheduler: Optional learning rate scheduler
        save_path: Optional path to save best model
    
    Returns:
        dict: Training history with losses and accuracies
    """

    model = model.to(device)

    history = {'train_loss': [], 'train_acc': [], 'val_loss': [], 'val_acc': []}

    best_val_acc = 0.0

    for epoch in range(num_epochs):
        print(f"Epoch {epoch + 1}/{num_epochs}")
        print('-' * 50)

        # Training
        model.train()
        train_loss = 0.0
        train_corrects = 0
        train_total = 0

        for batch_idx, (inputs, labels) in enumerate(train_loader):
            inputs, labels = inputs.to(device), labels.to(device)

            optimizer.zero_grad()
            
            outputs = model(inputs)
            loss_value = loss(outputs, labels)
            
            loss_value.backward()
            optimizer.step()

            train_loss += loss_value.item() * inputs.size(0)
            _, predicted = torch.max(outputs, 1)
            train_total += labels.size(0)
            train_corrects += (predicted == labels).sum().item()

            if (batch_idx + 1) % 10 == 0:
                print(f"Batch {batch_idx + 1}/{len(train_loader)}, Loss: {loss_value.item():.4f}")

        epoch_train_loss = train_loss / train_total
        epoch_train_acc = 100* train_corrects / train_total

        model.eval()
        val_loss = 0.0
        val_correct = 0
        val_total = 0

        with torch.no_grad():
            for inputs, labels in val_loader:
                inputs, labels = inputs.to(device), labels.to(device)

                outputs = model(inputs)
                loss_value = loss(outputs, labels)

                val_loss += loss_value.item() * inputs.size(0)
                _, predicted = torch.max(outputs, 1)
                val_total += labels.size(0)
                val_correct += (predicted == labels).sum().item()

        epoch_val_loss = val_loss / val_total
        epoch_val_acc = 100 * val_correct / val_total

        history['train_loss'].append(epoch_train_loss) 
        history['train_acc'].append(epoch_train_acc)
        history['val_loss'].append(epoch_val_loss)
        history['val_acc'].append(epoch_val_acc)

        print(f"Train Loss: {epoch_train_loss:.4f}, Train Acc: {epoch_train_acc:.2f}%")
        print(f"Val Loss: {epoch_val_loss:.4f}, Val Acc: {epoch_val_acc:.2f}%")

        if save_path and epoch_val_acc > best_val_acc:
            best_val_acc = epoch_val_acc
            torch.save({
                'epoch': epoch + 1,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'val_acc': best_val_acc,
                'history': history
            }, save_path)
            print(f'Best model saved with val acc: {best_val_acc:.2f}%')

    print('\n'+"="*50)
    print(f'Training complete. Best val acc: {best_val_acc:.2f}%')
    print("="*50)

    return history

