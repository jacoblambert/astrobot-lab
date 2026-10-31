import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// The backend runs in the dev container. astrobot-selected-dev publishes no ports, so point
// the proxy at its bridge IP: ASTROBOT_BACKEND=http://172.19.0.2:8080 npm run dev
// (tools/v2_demo.sh prints the exact command).
const backend = process.env.ASTROBOT_BACKEND ?? "http://127.0.0.1:8080";

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      "/api": backend,
      "/ws": {
        target: backend.replace(/^http/, "ws"),
        ws: true,
      },
    },
  },
});
