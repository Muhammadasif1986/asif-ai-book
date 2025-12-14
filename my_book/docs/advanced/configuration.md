---
sidebar_position: 1
---

# Advanced Configuration

This section covers advanced configuration options for your AI book project.

## Docusaurus Configuration

The `docusaurus.config.ts` file contains all the configuration for your site. Here are some advanced options:

### Custom Themes

You can create custom themes by extending the default theme:

```typescript
// src/theme/index.ts
import React from 'react';
import {themes as prismThemes} from 'prism-react-renderer';

export default {
  prism: {
    theme: prismThemes.github,
    darkTheme: prismThemes.dracula,
    additionalLanguages: ['python', 'json', 'bash'],
  },
};
```

### Plugin Configuration

Configure additional plugins for enhanced functionality:

```typescript
// docusaurus.config.ts
export default {
  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'community',
        path: 'community',
        routeBasePath: 'community',
      },
    ],
  ],
};
```

### Custom CSS

Add custom CSS in `src/css/custom.css` to override default styles:

```css
/* Custom styles for AI book documentation */
.hero--primary {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
}

/* Custom code block styling */
.docusaurus-highlight-code-line {
  background-color: rgba(0, 0, 0, 0.1);
  display: block;
  margin: 0 calc(-1 * var(--ifm-pre-padding));
  padding: 0 var(--ifm-pre-padding);
}
```

## Environment Variables

Use environment variables for different deployment environments:

```bash
# .env.production
ALGOLIA_APP_ID=your_app_id
ALGOLIA_API_KEY=your_api_key
ALGOLIA_INDEX_NAME=your_index_name
```

## Performance Optimization

Optimize your site's performance with these techniques:

- Use lazy loading for images
- Minimize bundle size with code splitting
- Optimize images and assets
- Use a CDN for static assets