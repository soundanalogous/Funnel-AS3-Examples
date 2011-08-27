import processing.pdf.*;
import processing.serial.*;

// ラインフィードのキャラクターコード
final int LF = 10;

// グラフを表示する際の上下左右のマージン
final float margin = 4;

// Arduinoから読み取ったサンプル数のカウント
int count = 0;

// シリアルポート
Serial serialPort;

// キャプチャしたデータ
Vector capturedData = new Vector();

// 保存したテンプレートデータのカウント
int savedTemplateCount = 0;

// テンプレートデータとして保存する文字列
String templateDataToSave = "";

void setup() {
  size(400, 200);

  // テキスト表示に使用するフォントを生成してロード
  PFont font = createFont("CourierNewPSMT", 18);
  textFont(font);

  // シリアルポートのリストを表示する
  println(Serial.list());

  // Mac OS Xでは0番目のポートがArduinoになる場合が多いが
  // Windowsでは最後のポートがArduinoである場合が多い
  // 必要に応じてポート番号をデフォルトの0から変更する
  serialPort = new Serial(this, Serial.list()[0], 9600);

  // シリアルポートからすでに受信しているデータがあればクリア
  serialPort.clear();

  // LFを受け取った時にserialEventが呼ばれるようにセット
  serialPort.bufferUntil(LF);

  // 描画条件をセットして背景を白く塗りつぶし、初期メッセージを表示
  fill(0);
  smooth();
  background(255);
  text("Press the button to start", 10, 20);
}

void draw() {
  //
}

// Serial.bufferUntil()でセットした文字を受け取ると呼ばれる
void serialEvent(Serial port) {
  // シリアルポートから受信済みのメッセージを読み出す
  String message = port.readString();

  // メッセージが空であれば以下の処理を行わずにリターン
  if (message == null) {
    return;
  }

  // タブでメッセージを分割
  String[] data = splitTokens(message, "\t");

  // 3 つのデータが含まれていたら
  if (data.length == 3) {
    // データを読み出してキャプチャしたデータを収める配列に追加
    int[] newData = new int[3];
    newData[0] = Integer.parseInt(trim(data[0]));
    newData[1] = Integer.parseInt(trim(data[1]));
    newData[2] = Integer.parseInt(trim(data[2]));
    capturedData.add(newData);
    count++;

    // 読み取り中に表示を更新
    background(255);
    fill(0);
    text("Reading: " + count, 10, 20);
  }
  // データが含まれている行でなければ
  else {
    // 最初の文字列がdataであれば
    if ("data\n".equals(data[0])) {
      // キャプチャを開始したと判断してデータを収める配列をクリア
      capturedData.clear();
    }

    // 最初の文字が改行文字であれば
    else if (data[0].charAt(0) == '\r') {
      // 読み取り終了と判断して表示すべきデータがあることを示すフラグをセット
      count = 0;

      // キャプチャしたデータを表示
      drawCapturedData();

      // スペースキーを押すことで保存されることを表示
      fill(0);
      text("Press the SPACE key to save", 10, 20);
    }
  }
}

// キーが押されたら
void keyPressed() {
  // 押されたキーがスペースであれば
  if (key == ' ') {
    // キャプチャしたデータをPDFに出力
    beginRecord(PDF, "Template_" + savedTemplateCount +
      ".pdf");
    drawCapturedData();
    endRecord();

    // データをヘッダとして保存する文字列に追加してカウントを更新
    addTemplateData();
    savedTemplateCount++;

    // 保存が完了したことを示すメッセージを表示
    fill(0);
    text("Saved", 10, 20);
  }
}

// キャプチャしたデータを表示する
void drawCapturedData() {
  // 背景を白で塗りつぶす
  background(255);

  // ウィンドウの右と下の値を求める
  int right = width - 1;
  int bottom = height - 1;

  // 横軸、縦軸、中心の値を示す線を描画する
  stroke(0);
  strokeWeight(1);
  line(margin, bottom - margin, right - margin,
  bottom - margin);
  line(margin, margin, margin, bottom - margin);
  strokeWeight(0.5);
  line(margin, height / 2, right - margin, height / 2);

  // 各軸のデータを順次描画する
  for (int j = 0; j < 3; j++) {
    // 描画条件を設定してカーブの描画を開始
    noFill();
    strokeWeight(1);
    beginShape();

    // 軸に応じて描画色を変更する
    if (j == 0) {
      stroke(255, 0, 0);
    }
    else if (j == 1) {
      stroke(0, 255, 0);
    }
    else if (j == 2) {
      stroke(0, 0, 255);
    }

    // キャプチャしたデータを1 つずつ描画する
    for (int i = 0; i < capturedData.size(); i++) {
      int[] data = (int[])capturedData.elementAt(i);
      float v = map(data[j], -16, 16,
      bottom - margin, margin);
      float h = map(i, 0, capturedData.size() - 1,
      margin, right - margin);
      curveVertex(h, v);
      if (i == 0 || i == (capturedData.size() - 1)) {
        curveVertex(h, v);
      }
    }

    // カーブ1 本の描画を終了
    endShape();
  }
}

// テンプレートデータを保存用に追加する
void addTemplateData() {
  // 保存用の文字列にジェスチャのカウントをコメントとして追加する
  templateDataToSave += " // Template " + savedTemplateCount;
  templateDataToSave += "\n {\n";

  // 各点のデータをC/C++/Javaの文法に沿って文字列に追加していく
  for (int i = 0; i < capturedData.size(); i++) {
    int[] data = (int[])capturedData.elementAt(i);
    templateDataToSave += " { ";
    templateDataToSave += data[0] + ", ";
    templateDataToSave += data[1] + ", ";
    templateDataToSave += data[2];
    templateDataToSave += " },\n";
  }
  templateDataToSave += " },\n";
}

// escキーを押してスケッチの実行を終了した時に以下が実行される
void stop() {
  // Arduinoでヘッダとして使用するための文字列を生成する
  String header = "";
  header += "#include <avr/pgmspace.h>\n";
  header += "\n";
  header += "const int numTemplates = ";
  header += savedTemplateCount + ";\n";
  header += "const int templateDataLength = ";
  header += capturedData.size() + ";\n";
  header += "const int numAxises = 3;\n";
  header += "\n";
  header += "const prog_char templateData";
  header += "[numTemplates][templateDataLength][numAxises]";
  header += "PROGMEM = {\n";
  header += templateDataToSave;
  header += "};";

  // スケッチのディレクトリにTemplateData.hとして保存する
  saveBytes("TemplateData.h", header.getBytes());

  // 元々実装されている終了処理を実行する
  super.stop();
}

