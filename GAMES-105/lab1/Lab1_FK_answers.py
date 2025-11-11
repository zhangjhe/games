import numpy as np
from scipy.spatial.transform import Rotation as R

class Joint:
    def __init__(self, name):
        self.idx = 0
        self.name = name
        self.children = []
        self.offset = None
        self.channel = None
        self.parent_idx = None




# class Motion:

def load_motion_data(bvh_file_path):
    """part2 辅助函数，读取bvh文件"""
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].startswith('Frame Time'):
                break
        motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            motion_data.append(np.array(data).reshape(1,-1))
        motion_data = np.concatenate(motion_data, axis=0)
    return motion_data


def _deserialize_offset(file):
    tokens = _parse_line(file) # read offset
    if tokens[0] == "OFFSET":
        return tokens[1:]
    else:
        raise SyntaxError("no joint offset! ")


def _deserialize_channel(file):
    tokens = _parse_line(file)

    if tokens[0] == "CHANNELS":
        return tokens[1:]
    else:
        raise SyntaxError(f"no joint channels! ")


def _parse_joint(joint_name, file, is_end=False):
    joint = Joint(joint_name)

    tokens = _parse_line(file) # reading '{'
    
    joint.offset = _deserialize_offset(file) # reading offsets
    
    if not is_end:
        joint.channel = _deserialize_channel(file) # reading channels

    while True:
        tokens = _parse_line(file) # reading next line
        if tokens[0] == "JOINT":
            joint.children.append(_parse_joint(tokens[1], file))
        elif tokens[0] == "End":
            joint.children.append(_parse_joint(joint.name + '_end', file, is_end=True))
        elif tokens[0] == "}":
            break
        else:
            raise SyntaxError(f"Joint definition must end with an child joint, end site or closing bracket.")

    return joint


def _parse_line(file):
    tokens = []
    while len(tokens) < 1:
        tokens = file.readline().strip().split()
        print(tokens)
    return tokens


def _from_hierarchy(joint, joint_name, joint_parent, joint_offset, parent_index=-1):
    current_index = len(joint_name)  # 当前节点的索引
    joint_name.append(joint.name)
    joint_parent.append(parent_index)
    joint_offset.append(joint.offset)

    for child in joint.children:
        _from_hierarchy(child, joint_name, joint_parent, joint_offset, current_index)

    return joint_name, joint_parent, joint_offset


def part1_calculate_T_pose(bvh_file_path):
    """请填写以下内容
    输入： bvh 文件路径
    输出:
        joint_name: List[str]，字符串列表，包含着所有关节的名字
        joint_parent: List[int]，整数列表，包含着所有关节的父关节的索引，根节点的父关节索引为-1
        joint_offset: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的偏移量

    Tips:
        joint_name顺序应该和bvh一致
    """
    with open(bvh_file_path, 'r') as bvh_file:
        
        tokens = _parse_line(bvh_file) # reading "Hierarchy"
        root_tokens = _parse_line(bvh_file) 
        root = _parse_joint('RootJoint', bvh_file)

    joint_name, joint_parent, joint_offset = _from_hierarchy(root, [], [], [])
    return joint_name, joint_parent, np.array(joint_offset)



def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    """请填写以下内容
    输入: part1 获得的关节名字，父节点列表，偏移量列表
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数
        frame_id: int，需要返回的帧的索引
    输出:
        joint_positions: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
        joint_orientations: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
    Tips:
        1. joint_orientations的四元数顺序为(x, y, z, w)
        2. from_euler时注意使用大写的XYZ
    """
    
    joint_positions = []
    joint_orientations = []
    frame_data = motion_data[frame_id]
    rots = []
    
    target = 0
    for joint in joint_name:
        if joint.endswith("_end"):
            rots.append(None)
        else:
            rots.append(R.from_euler("XYZ", frame_data[ 3 + target*3 : 3 + (target+1)*3 ]))
            target += 1


    # apply rotation to joints
    for i, joint in enumerate(joint_name):
        if i == 0:
            joint_orientations.append(rots[i])
            joint_positions.append(frame_data[:3])
            continue

        if joint.endswith('_end'):
            joint_orientations.append(joint_orientations[joint_parent[i]])
        else:
            joint_orientations.append(
                joint_orientations[joint_parent[i]] * rots[i]
            )

        Q = joint_orientations[i]
        l = np.array(joint_offset[i],dtype=np.float64)
        Ql = Q.apply(l)
        joint_positions.append(
            joint_positions[joint_parent[i]] +  Ql
        )

    for i, _ in enumerate(joint_orientations):
        joint_orientations[i] = joint_orientations[i].as_quat()


    return np.array(joint_positions), np.array(joint_orientations)


def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    """
    将 A-pose的bvh重定向到T-pose上
    输入: 两个bvh文件的路径
    输出: 
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数。retarget后的运动数据
    Tips:
        两个bvh的joint name顺序可能不一致哦(
        as_euler时也需要大写的XYZ
    """
    motion_data = None
    return motion_data
