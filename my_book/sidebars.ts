import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Define the tutorial sidebar manually for better organization
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Tutorial Basics',
      items: [
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-page',
        'tutorial-basics/deploy-your-site',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Tutorial Extras',
      items: [
        'tutorial-extras/manage-docs-versions',
        'tutorial-extras/translate-your-site',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Advanced Guides',
      items: [
        'advanced/configuration',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'API Documentation',
      items: [
        'api/introduction',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Features',
      items: [
        'features/search',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Book Content',
      items: [
        'module1-ros2/index',
        'module2-digital-twin/index',
        'module3-ai-robot-brain/index',
        'module4-vla/index',
      ],
      collapsed: false,
    },
    {
      type: 'doc',
      id: 'contributing',
      label: 'Contributing',
      className: 'menu-item--contributing',
    },
  ],
};

export default sidebars;
