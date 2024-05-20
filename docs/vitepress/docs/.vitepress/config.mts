//import { defineConfig } from 'vitepress'
import { withMermaid } from 'vitepress-plugin-mermaid'

// https://vitepress.dev/reference/site-config
//export default defineConfig({
export default withMermaid({
  title: 'GISNav',
  description: 'GISNav documentation',
  lastUpdated: true,
  cleanUrls: true,
  metaChunk: true,
  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    nav: [
      { text: 'Home', link: '/' },
      { text: 'Guide', link: '/README' },
      { text: 'API Reference', link: '/reference/index' }
    ],
    outline: {
      level: 'deep'
    },
    sidebar: {
      '/': [
        {
          text: 'Getting started',
          items: [
            { text: 'Install from Debian package', link: '/install-from-debian-package' },
            { text: 'GISNav CLI', link: '/gisnav-cli' },
            { text: 'Admin portal', link: '/admin-portal' },
            { text: 'Mock GPS demo', link: '/README' },
          ]
        },
        {
          text: 'Simulation',
          items: [
            { text: 'Deploy with Docker Compose', link: '/deploy-with-docker-compose' },
            { text: 'Raspberry Pi 5 & Pixhawk FMUv4', link: '/raspberry-pi-pixhawk' },
          ]
        },
        {
          text: 'Development',
          items: [
            { text: 'Install locally', link: '/install-locally' },
            { text: 'Deploy for development', link: '/deploy-for-development' },
            { text: 'System architecture', link: '/system-architecture' },
            { text: 'Run ROS nodes', link: '/ros-run-node' },
            { text: 'Remap ROS topics', link: '/remap-ros-topics' },
            { text: 'Run tests', link: '/test-gisnav' },
            { text: 'Generate documentation', link: '/generate-documentation' },
            { text: 'Create Debian distributable', link: '/create-debian' },
            { text: 'Setup GIS server', link: '/setup-gis-server' },
          ]
        },
        {
          items: [
            { text: 'Troubleshooting', link: '/troubleshooting' },
            { text: 'Glossary', link: '/glossary' },
            { text: 'License', link: '/LICENSE' },
          ]
        }
      ],
      '/reference/': [
        {
          text: 'Core nodes',
          items: [
            { text: 'BBoxNode', link: '/reference/public/bbox_node' },
            { text: 'GISNode', link: '/reference/public/gis_node' },
            { text: 'StereoNode', link: '/reference/public/stereo_node' },
            { text: 'PoseNode', link: '/reference/public/pose_node' },
          ]
        },
        {
          text: 'Extension nodes',
          items: [
            { text: 'UORBNode', link: '/reference/public/uorb_node' },
            { text: 'NMEANode', link: '/reference/public/nmea_node' },
            { text: 'QGISNode', link: '/reference/public/qgis_node' },
          ]
        },
        {
          text: 'Testing',
          items: [
            { text: 'Unit tests', link: '/reference/test/unit' },
            { text: 'Launch tests', link: '/reference/test/launch' },
          ]
        },
        {
          text: 'Private API',
          items: [
            { text: 'Decorators', link: '/reference/private/decorators' },
            { text: 'Transformations', link: '/reference/private/transformations' },
          ]
        },
        {
          items: [
            { text: 'Constants', link: '/reference/public/constants' },
            { text: 'Entry points', link: '/reference/public/gisnav' },
          ]
        },
      ]
    },

    socialLinks: [
      { icon: 'github', link: 'https://github.com/hmakelin/gisnav' }
    ],

    // logo: { src: '/assets/gisnav-website-favicon-color.png', width: 24, height: 24 },

    search: {
      provider: 'local'
    },

    editLink: {
      text: 'Edit this page on GitHub',
      pattern: 'https://github.com/hmakelin/gisnav/edit/main/docs/:path'
    },

    footer: {
      message: 'Released under the MIT License.',
      copyright: 'Copyright © 2022 Harri Makelin'
    },
  },
})
