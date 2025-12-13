---
sidebar_position: 2
---

# Create a Page

Pages are **React components** that can be created in the `src/pages` directory.

## Create your first Page

Create a React component at `src/pages/my-react-page.tsx`:

```jsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

export default function MyReactPage() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={`${siteConfig.title}`} description="Description goes here">
      <main>
        <div className="container padding-horiz--md">
          <h1>My React page</h1>
          <p>This is a React page</p>
          <p>
            <Link to="/">Go to homepage</Link>
          </p>
        </div>
      </main>
    </Layout>
  );
}
```

A new page is now available at [http://localhost:3000/my-react-page](http://localhost:3000/my-react-page).

## Create your first Markdown Page

Create a Markdown file at `src/pages/my-markdown-page.md`:

```md
---
title: My Markdown page
---

# My Markdown page

This is a Markdown page
```

A new page is now available at [http://localhost:3000/my-markdown-page](http://localhost:3000/my-markdown-page).