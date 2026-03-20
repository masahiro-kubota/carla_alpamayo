# carla_alpamayo Project Plan

このファイルは、AWS + NVIDIA の AV 3.0 パイプライン記事を踏まえて、`carla_alpamayo` で **個人レベルで体験できる縮小版プロジェクト**として何をやるかを整理したメモです。

## このディレクトリをどう使うか

`/home/masa/carla_alpamayo` は、このプロジェクトの **作業ルート兼モノレポ** として使う。

このディレクトリに置くもの:

- 設計メモ
- 設定ファイル
- 収集 / 整形 / 学習 / 評価のコード
- 軽量な manifest と sample
- 実験結果の要約

このディレクトリに置かないもの:

- `CARLA` 本体の巨大バイナリ
- 生の大量画像データ
- 使い捨ての一時ファイル

配置方針:

- `CARLA 0.9.16` 本体は `~/sim/carla-0.9.16` に置く
- `carla_alpamayo` 側には、その上で動くコードと設定だけを置く
- 生データや大きな生成物は `data/` や `outputs/` に逃がし、後で Git 管理するなら除外する

## 目的

目標は、企業向けの巨大な AV 3.0 基盤をそのまま再現することではなく、次のループを**個人でも回せる最小構成**で体験することです。

1. データを集める
2. クリップ化・説明付け・埋め込み化する
3. 欲しいシナリオを検索する
4. 小さな VLA 風モデルを学習する
5. 難しいケースで失敗させる
6. 失敗に似たケースを集め直して再学習する

面白さの本丸は、単に走れることではなく、**複雑なシチュエーションに対して、データ中心の反復で対応力が改善するか**を見ることです。

## 個人版での割り切り

- 実車フリートは使わない
- 最初は自分の車にカメラや RTK-GNSS を積まない
- `CARLA` を擬似フリート兼シミュレータとして使う
- 巨大な本物の VLA ではなく、**軽量 VLM / VLA 風モデル**から始める
- 企業向けの大規模サービス群は、ローカル実行しやすい部品に置き換える

## どんな構成で始めるか

一番おすすめの出発点はこれです。

- `CARLA`
- 軽量 VLM / VLA 風モデル
- ベクトル検索
- 1 リポジトリ構成

理由:

- 見た目とセンサーデータが自動運転らしい
- 実データなしで繰り返し収集できる
- `BehaviorAgent` や `Traffic Manager` を expert / traffic source の代わりに使える
- low quality 設定で個人 PC でも回しやすい

## 元記事のパイプラインとの対応

元記事では Stage 1 が「実車フリート収集」、Stage 8 が「シミュレータ検証」ですが、個人版では CARLA を両方に使います。

- Stage 1 相当
  - CARLA で擬似フリートデータを収集する
- Stage 2
  - ログの整形、クリップ分割、メタデータ付与
- Stage 3
  - caption と embedding を生成する
- Stage 4
  - ベクトル検索で hard cases を引けるようにする
- Stage 5
  - 必要なら軽い augmentation を入れる
- Stage 6
  - 最初は省略、または後回し
- Stage 7
  - 小さな VLA 風モデルを学習する
- Stage 8
  - CARLA closed-loop evaluation を回す

## 最初の成功条件

最初から「人間並みの汎化」を目指さない。まずは次を狙う。

- 1 つの Town 内でそこそこ走れる
- 難シナリオで baseline よりマシになる
- 失敗ケース検索 -> 再学習で改善が見える

評価レベルの目安:

1. 学習に使ったルートで走れる
2. 同じ Town の未学習ルートで走れる
3. 同じ Town で天候を変えても多少走れる
4. 別 Town に少しだけ広がる

最初の目標は `Level 2` で十分です。

## 難しいシナリオ集をどう作るか

難しいケースは自然発生を待つのではなく、**scenario family を定義して量産する**。

最初の候補:

- unprotected left turn
- pedestrian occlusion
- lead vehicle sudden brake
- merge with dense traffic
- blocked lane with forced avoidance

各 family ごとに変えるパラメータ:

- ego speed
- opponent speed
- pedestrian spawn timing
- traffic density
- weather
- time of day
- route seed

最初は:

- 5 family
- 各 family 20-50 seed
- 失敗した episode だけ `hard scenario bank` に昇格

## モデル構成の最小案

最初は VLA を厳密に再現しなくてもよい。まずは次のような入力で十分。

- front RGB
- speed
- high-level command
- ego state

出力:

- steer
- throttle
- brake

比較対象:

- baseline: 画像だけの軽量 policy
- VLA 風: 画像 + command
- 余力があれば: 画像 + command + short text context

## データパイプライン最小案

1. CARLA で episode を収集
2. 5 秒 clip に分割
3. clip ごとに metadata を保存
4. caption を生成
5. embedding を作る
6. vector DB に入れる
7. 失敗 clip に似た clip を検索する
8. hard dataset を再構築する

## リポジトリ方針

最初は **1 リポジトリでよい**。分けるのは困ってから。

候補構成:

```text
carla_alpamayo/
  README.md
  docs/
    PERSONAL_PROJECT_PLAN.md
  configs/
  apps/
    search-ui/
  pipelines/
    collect/
    curate/
    index/
    train/
    evaluate/
  libs/
    schemas/
    carla_utils/
    utils/
  scripts/
  data/
    sample/
    manifests/
  outputs/
    collect/
    curate/
    index/
    train/
    evaluate/
```

原則:

- 生データは Git に入れない
- stage ごとに入出力フォルダを分ける
- ログ形式は simulator 非依存に寄せる
- `CARLA` 本体そのものはリポジトリの外に置く

## carla_alpamayo の最初の使い方

最初は、`carla_alpamayo` を「CARLA 上で回す実験コード置き場」と割り切るのがよい。

順番:

1. `~/sim/carla-0.9.16` に `CARLA` 本体を入れる
2. `carla_alpamayo` にディレクトリ骨組みを作る
3. `pipelines/collect/` に最小のデータ収集コードを置く
4. `libs/schemas/` にログ schema を置く
5. `scripts/` に起動補助スクリプトを置く
6. 最初の収集対象を `Town01` か `Town03` に固定する

最初の骨組みを作るコマンド例:

```bash
cd /home/masa/carla_alpamayo

mkdir -p docs configs apps/search-ui \
  pipelines/{collect,curate,index,train,evaluate} \
  libs/{schemas,carla_utils,utils} \
  scripts \
  data/{sample,manifests} \
  outputs/{collect,curate,index,train,evaluate}
```

Git 管理を始めるなら:

```bash
cd /home/masa/carla_alpamayo
git init
```

Python 仮想環境を作るなら:

```bash
cd /home/masa/carla_alpamayo
python3 -m venv .venv
source .venv/bin/activate
```

最初の段階では、`README.md` に次だけ書ければ十分:

- どの `CARLA` バージョンを前提にするか
- どう起動するか
- どのコマンドで収集を始めるか
- 収集結果がどこに出るか

今の実体配置:

- 企画メモは `docs/PERSONAL_PROJECT_PLAN.md`
- ルートの `README.md` はセットアップと実行手順を書く場所
- 実装は `pipelines/` と `libs/` に置く

## CARLA を選ぶ理由

MetaDrive のほうが軽い可能性はあるが、最終的に CARLA の見た目とセンサーデータでやりたいなら、**最初から CARLA で始めるほうが二度手間が少ない**。

注意点:

- `no-rendering mode` は GPU センサーが空になるので、カメラ中心の VLA 用データ収集には向かない
- データ収集には `Low quality + RenderOffScreen` がよい

## CARLA バージョン方針

個人で始めるなら、まずは **UE4 系の packaged release** から始める。

- 2026-03-21 時点で GitHub Releases の latest は `CARLA 0.9.16`
- Quick start docs 本文には `0.9.15` と書かれている箇所があるが、配布ページでは `0.9.16` が最新

方針:

- まずは `CARLA 0.9.16`
- low quality で収集と評価を回す
- 後で必要になったら UE5 系を別環境で検討する

## CARLA インストール手順

### 1. 本体ダウンロード

```bash
mkdir -p ~/sim/carla-0.9.16
cd ~/Downloads

wget -O CARLA_0.9.16.tar.gz https://tiny.carla.org/carla-0-9-16-linux
tar -xzf CARLA_0.9.16.tar.gz -C ~/sim/carla-0.9.16
```

### 2. 追加マップ

```bash
cd ~/Downloads
wget -O AdditionalMaps_0.9.16.tar.gz https://tiny.carla.org/additional-maps-0-9-16-linux
mkdir -p ~/sim/carla-0.9.16/Import
mv AdditionalMaps_0.9.16.tar.gz ~/sim/carla-0.9.16/Import/
cd ~/sim/carla-0.9.16
./ImportAssets.sh
```

### 3. Python API インストール

```bash
cd ~/sim/carla-0.9.16/PythonAPI/carla/dist
python3 -m pip install ./carla-*.whl

cd ../examples
python3 -m pip install -r requirements.txt
```

### 4. 起動

通常起動:

```bash
cd ~/sim/carla-0.9.16
./CarlaUE4.sh
```

データ収集寄り:

```bash
cd ~/sim/carla-0.9.16
./CarlaUE4.sh -quality-level=Low -RenderOffScreen
```

### 5. 動作確認

交通生成:

```bash
cd ~/sim/carla-0.9.16/PythonAPI/examples
python3 generate_traffic.py
```

手動運転:

```bash
cd ~/sim/carla-0.9.16/PythonAPI/examples
python3 manual_control.py
```

## 最初の 2 週間でやること

### Phase 1

- CARLA 0.9.16 を入れる
- `carla_alpamayo` のディレクトリ骨組みを作る
- `Town01` か `Town03` で起動確認
- `BehaviorAgent` / `Traffic Manager` が動くことを確認
- front RGB + speed + command のログを保存する

### Phase 2

- 5 秒 clip 分割を作る
- metadata schema を決める
- 失敗 episode を保存する
- hard scenario bank の初版を作る

### Phase 3

- clip captioning を入れる
- embedding を作る
- vector search を入れる
- 類似 hard case を引けるようにする

### Phase 4

- 軽量 policy / VLA 風モデルを学習
- closed-loop evaluation を回す
- hard case を追加して再学習

## 最低限のログ schema

最初から simulator 依存を減らすため、内部表現は固定する。

- `episode_id`
- `frame_id`
- `town_id`
- `route_id`
- `weather_id`
- `timestamp`
- `front_rgb_path`
- `speed`
- `command`
- `steer`
- `throttle`
- `brake`
- `collision`
- `lane_invasion`
- `success`

保存先の初期案:

- frame 単位のログ: `data/manifests/episodes/<episode_id>.jsonl`
- クリップ単位のメタデータ: `data/manifests/clips/<clip_id>.json`
- 画像: `outputs/collect/<episode_id>/front_rgb/`

## やらないこと

当面はやらない。

- 実車収集
- RTK-GNSS 導入
- 本格的な 3D Neural Reconstruction
- 企業向け大規模 GPU クラスタ
- 最初から巨大 VLA を学習すること

## 参考リンク

- AWS article source:
  - https://aws.amazon.com/blogs/industries/building-an-end-to-end-physical-ai-data-pipeline-for-autonomous-vehicle-3-0-on-aws-with-nvidia/
- CARLA releases:
  - https://github.com/carla-simulator/carla/releases
- CARLA quick start:
  - https://carla.readthedocs.io/en/0.9.16/start_quickstart/
- CARLA rendering options:
  - https://carla.readthedocs.io/en/latest/adv_rendering_options/
- CARLA Traffic Manager:
  - https://carla.readthedocs.io/en/latest/adv_traffic_manager/
- CARLA agents:
  - https://carla.readthedocs.io/en/0.9.15/adv_agents/

## ひとことで言うと

`carla_alpamayo` は、**CARLA を擬似フリート兼シミュレータとして使い、軽量 VLA 風モデルとベクトル検索を組み合わせて、hard cases に強くなる反復ループを個人で体験するための作業ルート**として使う。
