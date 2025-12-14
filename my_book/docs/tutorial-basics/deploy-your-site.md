---
sidebar_position: 3
---

# Deploy your Site

Docusaurus is a **static-site-generator** (also called JAMstack).

It builds your site as simple **static HTML, JavaScript and CSS files**.

## Build the Site

Build your site with the following command:

```bash
npm run build
```

The static files are generated in the `build` folder.

## Deploy your Site

Test your production build locally:

```bash
npm run serve
```

The `serve` command will serve the contents of the `build` folder.

You can now deploy the contents of the `build` folder to any static file hosting service.

### Popular Static Hosts

- [GitHub Pages](https://pages.github.com/)
- [Vercel](https://vercel.com/)
- [Netlify](https://www.netlify.com/)
- [AWS S3](https://aws.amazon.com/s3/)
- [Cloudflare Pages](https://pages.cloudflare.dev/)

### GitHub Pages Example

To deploy to GitHub Pages:

1. Set the correct `baseUrl` in `docusaurus.config.ts`
2. Run `npm run deploy`

```bash
GIT_USER=<your-github-username> npm run deploy
```