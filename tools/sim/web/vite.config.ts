import react from "@vitejs/plugin-react"
import { defineConfig } from "vite"

// Relative base so the build works from any folder or host
export default defineConfig({
  base: "./",
  plugins: [react()],
  server: { port: 5195 },
})
