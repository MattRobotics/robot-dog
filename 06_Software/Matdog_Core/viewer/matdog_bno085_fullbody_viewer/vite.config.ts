import { defineConfig } from 'vitest/config';

/**
 * Standalone MATDOG BNO085 full-body viewer.
 *
 * No backend, no proxy, no framework plugin. Canonical geometry is staged
 * into public/canonical by scripts/sync_canonical_assets.mjs (npm predev /
 * prebuild), so Vite serves it as ordinary static assets.
 *
 * Web Serial requires a secure context: use http://localhost (dev server or
 * `vite preview`), not file://.
 */
export default defineConfig({
  base: './',
  server: {
    host: '127.0.0.1',
    port: 5183,
    strictPort: false,
  },
  preview: {
    host: '127.0.0.1',
    port: 5184,
    strictPort: false,
  },
  build: {
    target: 'es2022',
    outDir: 'dist',
    emptyOutDir: true,
  },
  test: {
    environment: 'node',
    include: ['tests/**/*.test.ts'],
  },
});
