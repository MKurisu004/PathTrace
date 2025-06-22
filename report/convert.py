import os
from PIL import Image

def convert_bmp_to_png(directory):
    count = 0
    for filename in os.listdir(directory):
        if filename.lower().endswith(".bmp"):
            bmp_path = os.path.join(directory, filename)
            png_path = os.path.splitext(bmp_path)[0] + ".png"
            try:
                with Image.open(bmp_path) as img:
                    img.save(png_path, format="PNG")
                os.remove(bmp_path)
                count += 1
                print(f"Converted and deleted: {filename} -> {os.path.basename(png_path)}")
            except Exception as e:
                print(f"Failed to convert {filename}: {e}")
    print(f"\nFinished. {count} file(s) converted.")

if __name__ == "__main__":
    folder_path = "./img"  # 你可以改成任何路径，例如 "./images"
    convert_bmp_to_png(folder_path)