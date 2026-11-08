import os
import yaml
from pathlib import Path
from utils.auxiliary import load_yaml, update_with_new_nfz, update_with_new_area_poi, update_with_new_uas
import math
from datetime import datetime


def load_problem_definition(
    file: str = str(
        Path(__file__).parent.resolve()
        / os.path.sep.join(["..", "cfg", "pd", "problem_definition.yaml"])
    ),
):
    """
    Loads the definition for the problem by first combining the contents of
    'mission.yaml' and 'uas.yaml' into 'problem_definition.yaml'. Then checks
    if 'new_nfz.yaml' exists, and updates 'problem_definition.yaml' by appending or
    overriding values as required.

    Args:
        file (str): The path to the YAML file containing the problem definition
                    (default is the default YAML file path).

    Returns:
        dict: The loaded problem definition as a dictionary.
    """

    # Define paths to the 'mission.yaml' and 'uas.yaml'
    mission_file = Path(file).parent / "mission.yaml"
    uas_file = Path(file).parent / "uas.yaml"

    # Ensure both mission.yaml and uas.yaml exist
    assert os.path.exists(mission_file), f"File not found: {mission_file}"
    assert os.path.exists(uas_file), f"File not found: {uas_file}"

    # Load mission.yaml and uas.yaml
    mission_data = load_yaml(str(mission_file))
    uas_data = load_yaml(str(uas_file))

    # Combine the dictionaries of mission.yaml and uas.yaml
    combined_data = {**mission_data, **uas_data}

    # Write the combined data into problem_definition.yaml
    with open(file, 'w', encoding='utf-8') as f:
        yaml.safe_dump(combined_data, f)

    # Now load the problem_definition.yaml that was created
    problem_data = load_yaml(file)

    # Check if new_nfz.yaml exists
    new_nfz_file = Path(file).parent / "new_nfz.yaml"
    if os.path.exists(new_nfz_file):
        new_nfz_data = load_yaml(str(new_nfz_file))
        # Update problem_definition.yaml using the data from new_nfz.yaml
        problem_data = update_with_new_nfz(problem_data, new_nfz_data)

        # Save the updated problem_definition.yaml
        with open(file, 'w', encoding='utf-8') as f:
            yaml.safe_dump(problem_data, f)

    # Check if new_area.yaml exists
    new_area_file = Path(file).parent / "new_area.yaml"
    if os.path.exists(new_area_file):
        new_area_data = load_yaml(str(new_area_file))
        # Update problem_definition.yaml using the data from new_area.yaml
        problem_data = update_with_new_area_poi(problem_data, new_area_data)

        # Save the updated problem_definition.yaml
        with open(file, 'w', encoding='utf-8') as f:
            yaml.safe_dump(problem_data, f)

    # Check if new_area.yaml exists
    new_uas_file = Path(file).parent / "new_uas.yaml"
    if os.path.exists(new_uas_file):
        new_uas_data = load_yaml(str(new_uas_file))
        # Update problem_definition.yaml using the data from new_area.yaml
        problem_data = update_with_new_uas(problem_data, new_uas_data)

        # Save the updated problem_definition.yaml
        with open(file, 'w', encoding='utf-8') as f:
            yaml.safe_dump(problem_data, f)
    
    # Return the final problem_definition.yaml data
    return problem_data


'''
def load_problem_definition(
    file=str(
        Path(__file__).parent.resolve()
        / os.path.sep.join(["..", "cfg", "problem_definition.yaml"])
    ),
):
    """
    Loads the definition for the problem

    Args:
        file (str): The path to the YAML file containing the
        problem definition (default is the default YAML file path).

    Returns:
        dict: The loaded problem definition as a dictionary."""
    return load_yaml(file)
'''

def load_solver_configuration(
    file=str(
        Path(__file__).parent.resolve()
        / os.path.sep.join(["..", "cfg", "solver_configuration.yaml"])
    )
):
    """
    Loads the configuration for the solver

    Args:
        file (str, optional): The file path of the configuration
        (default is the path to the solver_configuration.yaml file)

    Returns:
        dict: The loaded configuration as a dictionary"""
    return load_yaml(file)
