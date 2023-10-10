import numpy as np

ANGLE_INFO =  [('right_shoulder', ['right_hip', 'right_shoulder', 'right_elbow']),
           ('left_shoulder', ['left_hip', 'left_shoulder', 'left_elbow']),
           ('right_elbow', ['right_shoulder', 'right_elbow', 'right_wrist']),
           ('left_elbow', ['left_shoulder', 'left_elbow', 'left_wrist'])]
ANGLE_ORDER = {
    'right_shoulder': [0, 1, 2],
    'left_shoulder': [3, 4, 5],
    'right_elbow': [6, 7, 8],
    'left_elbow': [9, 10, 11],

}
EXERCISE_INFO = {
    'bicep_curls': {
        'segmenting_joints': [('right_shoulder', 2), 
                                ('left_shoulder', 2), 
                                ('right_elbow', 2),
                                ('left_elbow',  2)],
        'comparison_joints': [('right_shoulder', [0, 1, 2]), 
                                ('left_shoulder', [0, 1, 2]), 
                                ('right_elbow', [0, 1, 2]),
                                ('left_elbow',  [0, 1, 2])],
        'threshold1': 1500,
        'threshold2': 2000

    },
    'lateral_raises': {
        'segmenting_joints': [('right_shoulder', 2), 
                                ('left_shoulder', 2), 
                                ('right_elbow', 2),
                                ('left_elbow',  2),
                                ('right_shoulder', 0)],
        'comparison_joints': [('right_shoulder', [0, 1, 2]), 
                        ('left_shoulder', [0, 1, 2]), 
                        ('right_elbow', [0, 1, 2]),
                        ('left_elbow',  [0, 1, 2])],
        'threshold1': 1700,
        'threshold2': 2000
    }

}

EXPERTS = {}
for exercise_name in EXERCISE_INFO.keys():
    npzfile = np.load('src/quori_exercises/experts/{}_updated_experts.npz'.format(exercise_name), allow_pickle=True)

    EXPERTS[exercise_name] = {}
    EXPERTS[exercise_name]['experts']= npzfile['experts']
    EXPERTS[exercise_name]['duration'] = npzfile['expert_duration']
    EXPERTS[exercise_name]['labels'] = npzfile['labels']

    EXPERTS[exercise_name]['good'] = np.array([ii for ii, label in enumerate(EXPERTS[exercise_name]['labels']) if 'Good' in label]).astype(int)
