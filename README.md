# LEAP Hand Assets
The LEAP Hand assets include **URDF and USD files** for both **the right and left hands**. Since an official USD file for the left hand is not available, we generated one by **mirroring the right-hand** model. There is a subtle difference between the mirrored model and the model generated from the left-hand URDF. Specifically, the shapes of the first joints (following the order in URDF) on the index, middle, and ring fingers are asymmetrical. You can verify this by visualizing the mirrored USD file in Isaac Sim for comparison.

## leap_hand_right
[](img/leap_hand_left_urdf.png)
