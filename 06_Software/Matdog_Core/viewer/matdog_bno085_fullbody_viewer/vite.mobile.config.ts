import { defineConfig } from 'vite';

/**
 * Phone twin bundle (mobile.html) to be served BY THE ROBOT on its AP.
 * Relative base: the ESP32 serves it under /twin/. Single JS chunk, no
 * external fetch, no service worker (http://192.168.4.1 is not a secure
 * context; offline reuse relies on immutable HTTP caching of hashed names).
 */
export default defineConfig({
  base: './',
  // Never ship public/canonical (5.99 MB STL): the phone gets the MTW1 pack only.
  publicDir: false,
  build: {
    target: 'es2020',
    outDir: 'dist-mobile',
    emptyOutDir: true,
    assetsInlineLimit: 0,
    cssCodeSplit: false,
    modulePreload: false,
    // Fixed names: the ESP32 serves exactly these paths under /twin/ (ETag = SHA-256).
    rollupOptions: { input: 'mobile.html', output: { inlineDynamicImports: true, entryFileNames: 'assets/twin.js', assetFileNames: 'assets/[name][extname]' } },
  },
});
