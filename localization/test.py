import torchvision
from PIL import Image

from localization.localizer import Localizer

def test_localizer():
    localizer = Localizer()

    ground_img_path = "/Volumes/SALVATORE R/Università/TESP/data/ground_view_1.png"
    map_img_path = "/Volumes/SALVATORE R/Università/TESP/data/map_warped.jpeg"

    ground_img = Image.open(ground_img_path).convert('RGB')
    ground_img = torchvision.transforms.Resize((154, 231))(ground_img)
    ground_img = torchvision.transforms.ToTensor()(ground_img).unsqueeze(0)

    map_img = Image.open(map_img_path).convert('RGB')
    map_img = torchvision.transforms.Resize((512, 512))(map_img)
    map_img = torchvision.transforms.ToTensor()(map_img).unsqueeze(0)

    return localizer.get_pose_in_map(ground_img, map_img, (123, 123))


if __name__ == '__main__':
    test_localizer()
