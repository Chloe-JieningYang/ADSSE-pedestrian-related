#!/usr/bin/env python3
"""
启动本地 HTTP 服务器来运行 D3.js 仪表板
解决 CORS 问题，允许浏览器加载本地 GeoJSON 文件
"""

import http.server
import socketserver
import webbrowser
import os
import sys

PORT = 8000

class CORSRequestHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()

    def log_message(self, format, *args):
        # 自定义日志格式
        print(f"[{self.log_date_time_string()}] {format % args}")

def main():
    # 检查文件是否存在
    dashboard_path = 'MapVisual/index.html'
    if not os.path.exists(dashboard_path):
        print("❌ 错误: 找不到 MapVisual/index.html 文件")
        print("   请确保 MapVisual 文件夹存在且包含 index.html")
        sys.exit(1)
    
    if not os.path.exists('ca_martinez.geojson'):
        print("⚠️  警告: 找不到 ca_martinez.geojson 文件")
        print("   请确保 GeoJSON 文件在同一目录下")

    # 创建服务器
    with socketserver.TCPServer(("", PORT), CORSRequestHandler) as httpd:
        url = f"http://localhost:{PORT}/MapVisual/index.html"
        print("=" * 60)
        print("🚀 D3.js 仪表板服务器已启动")
        print("=" * 60)
        print(f"📊 访问地址: {url}")
        print(f"📁 服务目录: {os.getcwd()}")
        print("=" * 60)
        print("💡 提示:")
        print("   - 按 Ctrl+C 停止服务器")
        print("   - 浏览器将自动打开")
        print("=" * 60)
        
        # 自动打开浏览器
        try:
            webbrowser.open(url)
        except Exception as e:
            print(f"⚠️  无法自动打开浏览器: {e}")
            print(f"   请手动访问: {url}")

        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n\n🛑 服务器已停止")
            sys.exit(0)

if __name__ == "__main__":
    main()