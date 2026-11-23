import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// The backend runs in astrobot-dev, which publishes port 8080 (docker-compose.yml). Point the
// proxy elsewhere for a remote machine: ASTROBOT_BACKEND=http://<host>:8080 npm run dev
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
