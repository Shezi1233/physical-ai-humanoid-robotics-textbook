import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here

const config: Config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'Learning Physical AI and Humanoid Robotics for the future of robotics and AI',
  favicon: 'img/favicon.ico',

  // Important for Vercel deployment
  trailingSlash: false,

  future: {
    v4: true,
  },

  // ✅ VERCEL DEPLOYMENT - Updated with your actual live URL
  url: 'https://physical-ai-humanoid-robotics-textb-theta-lovat.vercel.app',
  baseUrl: '/',  // Root deployment ke liye '/' hi rakhna hai

  // GitHub Pages wale configs comment out kar diye kyunki ab Vercel use kar rahe ho
  // organizationName: 'Shezi1233',
  // projectName: 'Physical-AI-Humanoid-Robotics-Textbook',
  // deploymentBranch: 'gh-pages',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/Shezi1233/Physical-AI-Humanoid-Robotics-Textbook',
        },
        blog: {
          showReadingTime: true,
          editUrl:
            'https://github.com/Shezi1233/Physical-AI-Humanoid-Robotics-Textbook',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Robotics',
      logo: {
        alt: 'Physical AI and Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Tutorial',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/Shezi1233',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Tutorial',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
            {
              label: 'X',
              href: 'https://x.com/docusaurus',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/Shezi1233',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI and Humanoid Robotics Project. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;