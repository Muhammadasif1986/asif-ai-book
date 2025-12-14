# Docusaurus Chat Widget Integration Guide

This document outlines how to integrate the chat widget with a Docusaurus layout.

## Method 1: Using Docusaurus Theme Components

To integrate the chat widget with your Docusaurus site, you can wrap your layout with the ChatWidget component:

```jsx
// In your layout file (e.g., src/theme/Layout/index.js or src/pages/index.js)
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatWidget
        apiUrl={process.env.REACT_APP_API_URL || '/api/v1'}
        bookId="your-book-id"
      />
    </>
  );
}
```

## Method 2: Using Docusaurus MDX Components

You can also create a custom MDX component that includes the chat widget:

```jsx
// In src/theme/MDXComponents/index.js
import React from 'react';
import MDXComponents from '@theme-original/MDXComponents';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

export default {
  ...MDXComponents,
  ChatWidget
};
```

Then use it in your MDX files:

```mdx
<ChatWidget apiUrl="/api/v1" bookId="my-book" />

# My Content

This page has the chat widget integrated!
```

## Method 3: Direct Integration in Docusaurus Config

To add the chat widget globally, modify your `docusaurus.config.js` to inject the widget:

```js
// In docusaurus.config.js
module.exports = {
  // ... other config
  scripts: [
    // ... other scripts
    {
      src: '/js/chat-widget-loader.js',
      async: true,
    },
  ],
};
```

And create `static/js/chat-widget-loader.js`:

```js
// static/js/chat-widget-loader.js
import React from 'react';
import ReactDOM from 'react-dom/client';
import ChatWidget from '../../src/components/ChatWidget/ChatWidget';

// Wait for the page to load
window.addEventListener('load', () => {
  // Create a container for the chat widget
  const container = document.createElement('div');
  document.body.appendChild(container);

  // Render the chat widget
  const root = ReactDOM.createRoot(container);
  root.render(
    <ChatWidget
      apiUrl={window.CHAT_API_URL || '/api/v1'}
      bookId={window.BOOK_ID || 'default-book'}
    />
  );
});
```

## Styling Considerations

Make sure to include the necessary CSS for the chat widget. You can add this to your global CSS file:

```css
/* In src/css/custom.css */
.chat-widget-container {
  z-index: 9999;
}

.chat-message-user {
  background-color: #3b82f6;
  color: white;
}

.chat-message-bot {
  background-color: #e5e7eb;
  color: #374151;
}
```

## Environment Variables

Make sure to set the necessary environment variables in your `.env` file:

```
REACT_APP_API_URL=https://your-api-domain.com/api/v1
REACT_APP_BOOK_ID=your-book-identifier
```

## Testing the Integration

1. Start your Docusaurus development server: `npm run start`
2. Verify that the chat widget appears as a floating button in the bottom right corner
3. Click the button to open the chat interface
4. Test both general book queries and selection-based queries
5. Verify that the widget doesn't interfere with page navigation or content readability