package com.labatata.neskwik;

import android.content.ClipData;
import android.content.Intent;
import android.database.Cursor;
import android.net.Uri;
import android.provider.OpenableColumns;

import org.libsdl.app.SDLActivity;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;

public class NeskwikActivity extends SDLActivity {
    private static final String TAG = "NESkwik";

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (data != null) {
            persistUriPermissions(data);
        }
        super.onActivityResult(requestCode, resultCode, data);
    }

    public String getDisplayNameForUri(String uriText) {
        if (uriText == null || !uriText.startsWith("content://")) {
            return null;
        }

        Uri uri = Uri.parse(uriText);
        Cursor cursor = null;
        try {
            cursor = getContentResolver().query(
                    uri,
                    new String[] { OpenableColumns.DISPLAY_NAME },
                    null,
                    null,
                    null
                );
            cursor.moveToFirst();
            return cursor.getString(cursor.getColumnIndex(OpenableColumns.DISPLAY_NAME));
        } finally {
            if (cursor != null) {
                cursor.close();
            }
        }
    }

    private void persistUriPermissions(Intent data) {
        int permissionFlags = data.getFlags() &
            (Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION);

        getContentResolver().takePersistableUriPermission(data.getData(), permissionFlags);

        ClipData clipData = data.getClipData();
        if (clipData != null) {
            for (int i = 0; i < clipData.getItemCount(); i++) {
                getContentResolver().takePersistableUriPermission(clipData.getItemAt(i).getUri(), permissionFlags);
            }
        }
    }
}
