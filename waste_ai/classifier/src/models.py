import torch
import torch.nn as nn
import torchvision.models as models
import timm

def get_model(model_name: str, num_classes: int, pretrained: bool = True, freeze_backbone = True) -> nn.Module:
    
    """
    Get a pre-trained model and modify the final layer for the specified number of classes.

    Args:
        model_name (str): Name of the model architecture.
        num_classes (int): Number of output classes.
        pretrained (bool): Whether to load pre-trained weights.

    Returns:
        nn.Module: The modified model.
    """
    if model_name.lower() == 'resnet50':
        model = models.resnet50(weights=models.ResNet50_Weights.IMAGENET1K_V1 if pretrained else None)
          
        if freeze_backbone:
            for param in model.parameters():
                param.requires_grad = False
            for param in model.fc.parameters():
                param.requires_grad = True

        in_features = model.fc.in_features  
        model.fc = nn.Linear(in_features, num_classes)  

    elif model_name.lower() == 'efficientnet_b0':
        model = models.efficientnet_b0(weights=models.EfficientNet_B0_Weights.IMAGENET1K_V1 if pretrained else None)


        if freeze_backbone:
            for param in model.features.parameters():
                param.requires_grad = False

        linear_layer = model.classifier[1]
        if isinstance(linear_layer, nn.Linear):
            in_features = model.classifier[1].in_features
            model.classifier[1] = nn.Linear(in_features, num_classes)
        else:
            raise ValueError("Expected classifier to be nn.Linear")

    else:
        raise ValueError("Model name not recognized. Supported models: 'resnet50', 'efficientnet_b0'.")
     
    return model
