import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/docs',
    component: ComponentCreator('/docs', '20a'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '3e9'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '29e'),
            routes: [
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/exercises',
                component: ComponentCreator('/docs/isaac-sim/exercises', '158'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/introduction',
                component: ComponentCreator('/docs/isaac-sim/introduction', 'b1a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/performance',
                component: ComponentCreator('/docs/isaac-sim/performance', '882'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/prerequisites',
                component: ComponentCreator('/docs/isaac-sim/prerequisites', 'dc4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/setup',
                component: ComponentCreator('/docs/isaac-sim/setup', '1ce'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/synthetic-data-generation',
                component: ComponentCreator('/docs/isaac-sim/synthetic-data-generation', 'c8d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/test-report',
                component: ComponentCreator('/docs/isaac-sim/test-report', '15f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/troubleshooting',
                component: ComponentCreator('/docs/isaac-sim/troubleshooting', '0af'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/validation-checklist',
                component: ComponentCreator('/docs/isaac-sim/validation-checklist', 'da9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/workflows',
                component: ComponentCreator('/docs/isaac-sim/workflows', 'd77'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/python-ros-control',
                component: ComponentCreator('/docs/module1/python-ros-control', '6ff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/ros2-core-concepts',
                component: ComponentCreator('/docs/module1/ros2-core-concepts', '2bc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/solutions/exercise1_solution',
                component: ComponentCreator('/docs/module1/solutions/exercise1_solution', 'a7d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/solutions/exercise2_solution',
                component: ComponentCreator('/docs/module1/solutions/exercise2_solution', 'c9c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/solutions/exercise3_solution',
                component: ComponentCreator('/docs/module1/solutions/exercise3_solution', '786'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/solutions/exercise4_solution',
                component: ComponentCreator('/docs/module1/solutions/exercise4_solution', '981'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/solutions/exercise5_solution',
                component: ComponentCreator('/docs/module1/solutions/exercise5_solution', 'dfc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/solutions/exercise6_solution',
                component: ComponentCreator('/docs/module1/solutions/exercise6_solution', '303'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/urdf-humanoids',
                component: ComponentCreator('/docs/module1/urdf-humanoids', '958'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/gazebo-physics-simulation',
                component: ComponentCreator('/docs/module2/gazebo-physics-simulation', '4f8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/sensor-simulation',
                component: ComponentCreator('/docs/module2/sensor-simulation', '793'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/solutions/sensor_simulation_exercise_solutions',
                component: ComponentCreator('/docs/module2/solutions/sensor_simulation_exercise_solutions', '6f4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/unity-rendering-hri',
                component: ComponentCreator('/docs/module2/unity-rendering-hri', '683'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/exercises',
                component: ComponentCreator('/docs/vla/exercises', '13e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/introduction',
                component: ComponentCreator('/docs/vla/introduction', 'bef'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/latency-accuracy',
                component: ComponentCreator('/docs/vla/latency-accuracy', 'fdd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/performance',
                component: ComponentCreator('/docs/vla/performance', 'c70'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/prerequisites',
                component: ComponentCreator('/docs/vla/prerequisites', 'bb2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/troubleshooting',
                component: ComponentCreator('/docs/vla/troubleshooting', '748'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/validation-report',
                component: ComponentCreator('/docs/vla/validation-report', 'af3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/voice-command-mapping',
                component: ComponentCreator('/docs/vla/voice-command-mapping', 'ac8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/whisper-integration',
                component: ComponentCreator('/docs/vla/whisper-integration', '644'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
