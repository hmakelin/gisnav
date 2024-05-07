//import { defineConfig } from 'vitepress'
import { withMermaid } from 'vitepress-plugin-mermaid'

// https://vitepress.dev/reference/site-config
//export default defineConfig({
export default withMermaid({
  title: "GISNav",
  description: "GISNav documentation",
  lastUpdated: true,
  cleanUrls: true,
  metaChunk: true,
  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    nav: [
      { text: 'Home', link: '/' },
      { text: 'Guide', link: '/README' },
      { text: 'API Reference', link: '/api-reference' }
    ],
    outline: {
      level: 'deep'
    },
    sidebar: [
      {
        text: 'SITL simulation',
        items: [
          { text: 'Run mock GPS demo', link: '/README' },
          { text: 'Deploy with Docker Compose', link: '/deploy-with-docker-compose' },
        ]
      },
      {
        text: 'HIL simulation',
        items: [
          { text: 'Jetson & Pixhawk', link: '/jetson-pixhawk' }
        ]
      },
      {
        text: 'Configuration',
        items: [
          { text: 'Admin portal', link: '/admin-portal' },
          { text: 'Setup GIS server', link: '/setup-gis-server' },
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
