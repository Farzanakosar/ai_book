// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'intro',
      ],
      link: {
        type: 'doc',
        id: 'intro',
      },
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/ros2-core-concepts',
        'module1/python-ros-control',
        'module1/urdf-humanoids',
        {
          type: 'category',
          label: 'Solutions',
          items: [
            'module1/solutions/exercise1_solution',
            'module1/solutions/exercise2_solution',
            'module1/solutions/exercise3_solution',
            'module1/solutions/exercise4_solution',
            'module1/solutions/exercise5_solution',
            'module1/solutions/exercise6_solution',
          ],
        },
      ],
      link: {
        type: 'doc',
        id: 'module1/ros2-core-concepts',
      },
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments (Gazebo/Unity)',
      items: [
        'module2/gazebo-physics-simulation',
        'module2/sensor-simulation',
        'module2/unity-rendering-hri',
        {
          type: 'category',
          label: 'Solutions',
          items: [
            'module2/solutions/sensor_simulation_exercise_solutions',
          ],
        },
      ],
      link: {
        type: 'doc',
        id: 'module2/gazebo-physics-simulation',
      },
    },
    {
      type: 'category',
      label: 'Isaac Sim',
      items: [
        'isaac-sim/introduction',
        'isaac-sim/prerequisites',
        'isaac-sim/setup',
        'isaac-sim/workflows',
        'isaac-sim/synthetic-data-generation',
        'isaac-sim/performance',
        'isaac-sim/troubleshooting',
        'isaac-sim/test-report',
        'isaac-sim/validation-checklist',
        'isaac-sim/exercises',
      ],
      link: {
        type: 'doc',
        id: 'isaac-sim/introduction',
      },
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA) Systems',
      items: [
        'vla/introduction',
        'vla/prerequisites',
        'vla/whisper-integration',
        'vla/voice-command-mapping',
        'vla/latency-accuracy',
        'vla/troubleshooting',
        'vla/performance',
        'vla/exercises',
        'vla/validation-report',
      ],
      link: {
        type: 'doc',
        id: 'vla/introduction',
      },
    },
  ],
};

module.exports = sidebars;
