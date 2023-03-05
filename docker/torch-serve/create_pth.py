import torch
from kornia.feature import LoFTR

model = LoFTR(pretrained='outdoor')
torch.save(model.state_dict(), 'loftr.pth')
