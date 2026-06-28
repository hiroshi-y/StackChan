// Smart HAMLOG app の Worker(WebSocket)接続設定 テンプレート。
//
// 使い方: このファイルを同じフォルダに cat_config.h としてコピーし、CAT_PAIR_TOKEN に
//         Web アプリのペアリングで生成した端末固有トークンを貼る。
//         cat_config.h は .gitignore 済み(実トークンを git に入れないこと)。
//         cat_config.h が無ければビルドはこのテンプレート(未設定トークン)で通る。
#pragma once

// worker-cat の WSS ホスト名(スキーム/パスは付けない)。
#define CAT_WORKER_HOST  "staging-smart-hamlog-cat.hiroshi-a4b.workers.dev"

// ペアリングトークン(Web アプリで生成した端末固有の値)。
#define CAT_PAIR_TOKEN   "PASTE_PAIR_TOKEN_HERE"
