import os
import xml.etree.ElementTree as ET
import yaml


def test_srdf_and_kinematics():
    config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
    srdf_path = os.path.join(config_dir, 'birobot.srdf')
    kinematics_path = os.path.join(config_dir, 'kinematics.yaml')

    assert os.path.exists(srdf_path), f"SRDF file not found at {srdf_path}"
    assert os.path.exists(kinematics_path), f"Kinematics yaml not found at {kinematics_path}"

    # Parse SRDF XML
    tree = ET.parse(srdf_path)
    root = tree.getroot()

    assert root.tag == 'robot'
    assert root.attrib.get('name') == 'birobot'

    groups = {}
    for group in root.findall('group'):
        group_name = group.attrib.get('name')
        groups[group_name] = group

    assert 'arm_1' in groups, "Planning group arm_1 missing from SRDF"
    assert 'arm_2' in groups, "Planning group arm_2 missing from SRDF"
    assert 'dual_arms' in groups, "Planning group dual_arms missing from SRDF"
    assert 'arm1_hand' in groups, "Planning group arm1_hand missing from SRDF"
    assert 'arm2_hand' in groups, "Planning group arm2_hand missing from SRDF"
    assert len(groups) == 5, f"Expected exactly 5 planning groups, found {len(groups)}"

    # Check arm_1 chain
    arm_1_chain = groups['arm_1'].find('chain')
    assert arm_1_chain is not None, "arm_1 group missing chain element"
    assert arm_1_chain.attrib.get('base_link') == 'arm1_base_link'
    assert arm_1_chain.attrib.get('tip_link') == 'arm1_tool0'

    # Check arm_2 chain
    arm_2_chain = groups['arm_2'].find('chain')
    assert arm_2_chain is not None, "arm_2 group missing chain element"
    assert arm_2_chain.attrib.get('base_link') == 'arm2_base_link'
    assert arm_2_chain.attrib.get('tip_link') == 'arm2_tool0'

    # Check dual_arms sub-groups
    subgroups = [g.attrib.get('name') for g in groups['dual_arms'].findall('group')]
    assert 'arm_1' in subgroups and 'arm_2' in subgroups, "dual_arms must compose arm_1 and arm_2"

    # Check end effectors
    ees = root.findall('end_effector')
    ee_names = [ee.attrib.get('name') for ee in ees]
    assert 'arm1_ee' in ee_names and 'arm2_ee' in ee_names
    assert 'arm1_gripper' in ee_names and 'arm2_gripper' in ee_names

    # Check self-collision matrix disable_collisions entries
    disabled = root.findall('disable_collisions')
    assert len(disabled) > 0, "No disable_collisions entries found in SRDF"

    has_inter_arm = False
    for dc in disabled:
        l1 = dc.attrib.get('link1', '')
        l2 = dc.attrib.get('link2', '')
        if (l1.startswith('arm1_') and l2.startswith('arm2_')) or (l1.startswith('arm2_') and l1.startswith('arm1_')):
            has_inter_arm = True
            break
    assert has_inter_arm, "Self-collision matrix must contain inter-arm disable_collisions entries"

    # Check Kinematics YAML
    with open(kinematics_path, 'r') as f:
        kinematics_data = yaml.safe_load(f)

    assert 'arm_1' in kinematics_data, "arm_1 missing from kinematics.yaml"
    assert 'arm_2' in kinematics_data, "arm_2 missing from kinematics.yaml"
    assert 'dual_arms' not in kinematics_data, "dual_arms should NOT have a dedicated solver in kinematics.yaml"

    assert kinematics_data['arm_1']['kinematics_solver'] == 'kdl_kinematics_plugin/KDLKinematicsPlugin'
    assert kinematics_data['arm_2']['kinematics_solver'] == 'kdl_kinematics_plugin/KDLKinematicsPlugin'

    print("SRDF AND KINEMATICS CONFIGURATION TESTS PASSED")


if __name__ == '__main__':
    test_srdf_and_kinematics()
