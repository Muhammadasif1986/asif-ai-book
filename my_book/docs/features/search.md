---
sidebar_position: 10
---

# Search Functionality

The Asif AI Book documentation includes powerful search capabilities to help you find the information you need quickly.

## Algolia Search

Our documentation uses Algolia DocSearch to provide fast and accurate search results across all documentation pages.

### Search Features

- **Full-text search**: Search across all documentation content
- **Instant results**: Get results as you type
- **Relevance ranking**: Most relevant results appear first
- **Faceted search**: Filter results by category or section
- **Offline support**: Search works even when browsing offline

### Search Tips

1. **Use specific terms**: Search for specific keywords rather than general concepts
2. **Try different phrasings**: If you don't find what you're looking for, try rephrasing your query
3. **Use quotes**: Wrap phrases in quotes for exact matches
4. **Check spelling**: Misspellings can affect search results

### Searchable Content

The search function covers:

- All documentation pages
- Code examples and snippets
- API references
- Tutorials and guides
- Configuration options

## Custom Search

For more advanced search needs, you can implement custom search functionality in your local development environment:

```bash
# Install search dependencies
npm install @docusaurus/theme-search-algolia

# Configure in docusaurus.config.ts
module.exports = {
  themeConfig: {
    algolia: {
      // The application ID provided by Algolia
      appId: 'YOUR_APP_ID',

      // Public API key: it is safe to commit it
      apiKey: 'YOUR_SEARCH_API_KEY',

      indexName: 'your-index-name',

      // Optional: see doc section below
      contextualSearch: true,

      // Optional: Specify domains where the navigation should occur through window.location instead on history.push. Useful when our Algolia config crawls multiple documentation sites and we want to navigate with window.location.href to them.
      externalUrlRegex: 'external\\.example\\.com|thirdparty\\.example\\.com',

      // Optional: Replace parts of the item URLs from Algolia. Useful when using the same search index for multiple deployments using a different baseUrl. You can use regexp or string in the `from` param. For example: localhost:3000 vs myCompany.com/docs
      replaceSearchResultPathname: {
        from: '/docs/', // or as RegExp: /\/docs\//
        to: '/',
      },

      // Optional: Algolia search parameters
      searchParameters: {},

      // Optional: path for search page that enabled by default (`false` to disable it)
      searchPagePath: 'search',
    },
  },
};
```

## Offline Search

When the documentation is deployed in environments where Algolia search is not available, you can enable local search:

```typescript
// docusaurus.config.ts
module.exports = {
  themes: [
    [
      '@docusaurus/theme-classic',
      {
        customCss: require.resolve('./src/css/custom.css'),
      },
    ],
    [
      '@docusaurus/theme-search-algolia',
      {
        // Local search configuration
        contextualSearch: true,
      },
    ],
  ],
};
```

## Search Analytics

Documentation maintainers can track search usage to improve content:

- Most searched terms
- Zero-result searches
- Popular content paths
- User search patterns