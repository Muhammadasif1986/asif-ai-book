---
sidebar_position: 1
---

# Create a Document

Documents are **groups of pages** connected through:

- a **sidebar** (a menu)
- **previous/next navigation**
- **versioning**

## Create your first Doc

Create a Markdown file at `docs/tutorial-basics/create-a-document.md`:

```md
---
sidebar_position: 1
---

# My First Document

This is my first document!
```

## Configure the Sidebar

Docusaurus automatically **creates a sidebar** from the `docs` folder.

You can also create your own sidebar by editing `sidebars.ts`:

```js
export default {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro', // document with id 'intro' in 'docs/intro.md'
    },
  ],
};
```