from cf_msgs.msg import DetectionResult, SignLabel


def sign_label_msg_to_dict_entry(sign_label_msg):
    """
     Each dictionary contains the following keys:
                - "x": Top-left corner column
                - "y": Top-left corner row
                - "width": Width of bounding box in pixel
                - "height": Height of bounding box in pixel
                - "category": Category (not implemented yet!)
                 """
    entry = {}
    entry["x"] = sign_label_msg.bbx_top_left_x
    entry["y"] = sign_label_msg.bbx_top_left_y
    entry["width"] = sign_label_msg.bbx_width
    entry["height"] = sign_label_msg.bbx_height
    entry["category"] = sign_label_msg.category_id
    entry["category_conf"] =sign_label_msg.category_conf

    return entry

def sign_label_msg_array_to_dict_list(sign_label_msg_array):
    """
     Each dictionary contains the following keys:
                - "x": Top-left corner column
                - "y": Top-left corner row
                - "width": Width of bounding box in pixel
                - "height": Height of bounding box in pixel
                - "category": Category (not implemented yet!)
                 """
    dict_list = []
    for entry in sign_label_msg_array:
        dict_list.append(sign_label_msg_to_dict_entry(entry))
   

    return dict_list

def dict_entry_to_sign_label_msg(entry):
    """
     Each dictionary contains the following keys:
                - "x": Top-left corner column
                - "y": Top-left corner row
                - "width": Width of bounding box in pixel
                - "height": Height of bounding box in pixel
                - "category": Category (not implemented yet!)
                 """
    sign_label_msg = SignLabel()
    sign_label_msg.bbx_top_left_x = entry["x"]
    sign_label_msg.bbx_top_left_y = entry["y"]
    sign_label_msg.bbx_width = entry["width"] 
    sign_label_msg.bbx_height = entry["height"] 
    sign_label_msg.category_id = entry["category"]
    sign_label_msg.category_conf = entry["category_conf"]

    return sign_label_msg
