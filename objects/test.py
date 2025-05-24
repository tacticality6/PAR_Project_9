import cv2, glob

for path in glob.glob("aruco*.png"):
    gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    for name in ["DICT_4X4_50","DICT_4X4_250","DICT_5X5_100","DICT_APRILTAG_36h11"]:
        D = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        ids = cv2.aruco.detectMarkers(gray, D)[1]
        print("  ❌ failed to load image", ids)
        if ids is not None:
            print(f"{path}: {name}  →  IDs {ids.flatten().tolist()}")
            
# import cv2
# import glob

# # 1) grab all your marker images
# files = glob.glob("aruco*.png")
# if not files:
#     raise RuntimeError("No aruco_*.png files found in the current folder!")

# # 2) the dicts to try
# dict_names = [
#     "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250",
#     "DICT_5X5_100", "DICT_6X6_250", "DICT_APRILTAG_36h11"
# ]

# for img_path in files:
#     print(f"\n=== Testing {img_path} ===")
#     # load as grayscale
#     img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
#     if img is None:
#         print("  ❌ failed to load image")
#         continue

#     # add a white border so detectMarkers can find the outer contour
#     pad = 20
#     img_padded = cv2.copyMakeBorder(
#         img, pad, pad, pad, pad,
#         borderType=cv2.BORDER_CONSTANT,
#         value=255
#     )

#     for name in dict_names:
#         # get the ArUco dictionary object
#         aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, name))

#         # detect on the padded image
#         corners, ids, _ = cv2.aruco.detectMarkers(img_padded, aruco_dict)

#         if ids is not None:
#             print(f"  ✔ {name} → detected IDs {ids.flatten().tolist()}")
#         else:
#             print(f"  ✘ {name} → no markers")
